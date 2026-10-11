#include "threads.hpp"

#include "BMI088.hpp"
#include "main.h" //< LED_R/G/B 引脚宏（LED 是 README 允许线程直接调 HAL GPIO 的例外）

/* ==================================================================== *
 * 方向 C · imu 线程（TODO #10 的 imu 部分）
 *
 * 职责：BMI088 初始化（芯片 ID 自检 → Config → Calibrate），随后按周期读三轴
 *       加速度/角速度/温度并做通信自检；数据放在 g_imu 里供其它线程读。
 *
 * 现状：C1（读通路 + ID 自检）、C2（数据解析 + 单位换算 + 通信自检）已完成。
 *       数据共享的互斥与 imu_alive 信号量属方向 E，这里先不做（单写者约定）。
 * ==================================================================== */

/* ---------------- 上板调试开关（风格同 main.c 的 BSP_SELF_TEST） ----------------
 * 1 = 用三色灯显示 IMU 状态：
 *       绿 = 芯片 ID 都对、且数据没被判定为通信失效（正常运行态）；
 *       红 = 芯片 ID 不对（总线/片选/供电级故障，此时跳过 Config/Calibrate）；
 *       蓝 = ID 对但数据被判通信失效（拔掉或短接 MISO 就能复现，用于验收 C2）。
 * 0 = 不驱动灯（方向 E 做出独立 LED 模块后，这段应删掉换成 LED 模块）*/
#define IMU_SELF_TEST_LED 1

/* ---------------- 线程参数 ---------------- */
#define IMU_THREAD_STACK_SIZE 1024U
#define IMU_THREAD_PRIORITY   10U /* 比 control(8) 低：IMU 晚一拍不影响电机安全 */
#define IMU_THREAD_TICK       1U  /* 1 tick = 1 ms，与 acc 800 Hz / gyro 2000 Hz 的采样率相称 */

static TX_THREAD imu_thread;
/* ThreadX 要求线程栈 8 字节对齐 */
static UCHAR imu_thread_stack[IMU_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/* 唯一的 IMU 实例：imu 线程写、其它线程读（单写者约定，暂不加互斥）。
 * 放静态存储而不是线程栈 —— 生命周期要覆盖整个运行期。
 * 故意不加 static：符号名就是 g_imu，上板时可以直接用 SWD 按符号地址读它。*/
BMI088::cBMI088 g_imu;

static void ImuThread_Entry(ULONG thread_input);

void ImuThread_Init(void)
{
    (void)tx_thread_create(&imu_thread,
                           (CHAR *)"imu",
                           ImuThread_Entry,
                           0,
                           imu_thread_stack,
                           IMU_THREAD_STACK_SIZE,
                           IMU_THREAD_PRIORITY,
                           IMU_THREAD_PRIORITY, /* 抢占门限 = 自身优先级 */
                           TX_NO_TIME_SLICE,
                           TX_AUTO_START);
}

#if IMU_SELF_TEST_LED
/* 三色灯刷新：红 = ID 错、蓝 = 数据错、绿 = 全好。
 * 放在主循环里每轮刷新，这样拔掉 MISO 之后能立刻看到灯变蓝（C2 验收要求）。*/
static void ImuLedUpdate(void)
{
    const bool id_err = g_imu.self_test.ACC_CHIP_ID_ERR || g_imu.self_test.GYRO_CHIP_ID_ERR;
    const bool data_err = g_imu.self_test.ACC_DATA_ERR || g_imu.self_test.GYRO_DATA_ERR;

    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, id_err ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (!id_err && !data_err) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, (!id_err && data_err) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif

/* BMI088 的经典坑：ACC_PWR_CTRL / ACC_PWR_CONF 这两次写**不保证一次生效**。
 * 本工程实测（见 docs/logs.md 第四篇）：Config() 里写完 ACC_PWR_CTRL=0x04 之后回读，
 * 值仍是 0x00 —— 加速度计一直停在复位默认的 suspend，三轴数据寄存器恒为 0x00。
 * 阴险的地方是芯片 ID 和温度都照常可读（温度不受 suspend 影响），
 * 所以很容易误判成“总线是好的，只是数据不对”。
 * 官方驱动的对策就是“写 → 回读 → 不匹配就重试”，这里照做。*/
#define ACC_POWER_RETRY_TIMES 10U
static void EnsureAccPowered(void)
{
    for (uint8_t i = 0U; i < ACC_POWER_RETRY_TIMES; ++i)
    {
        uint8_t cmd = ACC_PWR_CTRL_ON; // 0x04 = 加速度计上电
        g_imu.WriteReg(BMI088::BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &cmd, 1U);
        tx_thread_sleep(5);

        uint8_t rd[2] = {0U, 0U};
        g_imu.ReadReg(BMI088::BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, rd, 2U);
        if (rd[1] == ACC_PWR_CTRL_ON) break; // 回读一致才算真的写进去
    }

    for (uint8_t i = 0U; i < ACC_POWER_RETRY_TIMES; ++i)
    {
        uint8_t cmd = ACC_PWR_CONF_ACT; // 0x00 = active（脱离 suspend）
        g_imu.WriteReg(BMI088::BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &cmd, 1U);
        tx_thread_sleep(5);

        uint8_t rd[2] = {0U, 0U};
        g_imu.ReadReg(BMI088::BMI088_CS_ACC, ACC_PWR_CONF_ADDR, rd, 2U);
        if (rd[1] == ACC_PWR_CONF_ACT) break;
    }
}

static void ImuThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    /* 先给芯片和总线一点稳定时间（老师给的 Config() 也是这么开头的）。
     * 注意这几个读函数内部有 tx_thread_sleep，只能在内核启动之后（即线程里）调用。*/
    tx_thread_sleep(10);

    /* 热身读：重置后**第一次** SPI 事务实测出现过一次无效读（返回流里没有 0x1E），
     * 之后每次访问都正常。故这里先做一次丢弃读，把“第一次”消耗掉。*/
    uint8_t warmup[2] = {0U, 0U};
    g_imu.ReadReg(BMI088::BMI088_CS_ACC, ACC_CHIP_ID_ADDR, warmup, 2U);

    /* 校验两只芯片的 ID：ACC 期望 0x1E、GYRO 期望 0x0F。
     * 偶发误读不该直接判死，故失败时再重读一轮。*/
    g_imu.VerifyAccChipID();
    g_imu.VerifyGyroChipID();
    if (g_imu.self_test.ACC_CHIP_ID_ERR || g_imu.self_test.GYRO_CHIP_ID_ERR)
    {
        g_imu.VerifyAccChipID();
        g_imu.VerifyGyroChipID();
    }

    const bool id_ok = (!g_imu.self_test.ACC_CHIP_ID_ERR) && (!g_imu.self_test.GYRO_CHIP_ID_ERR);

    /* ID 过了才动寄存器：总线都不通就去软复位/写量程没有意义，
     * 而且 Config() 里的软复位会把刚确认过的状态清掉。*/
    if (id_ok)
    {
        g_imu.Config();     // 软复位 -> 上电 -> 量程/带宽，内部累计约 445 ms
        EnsureAccPowered(); // 再校验一次“上电/退出 suspend”是否真的写进去了（见上面的说明）

        g_imu.Calibrate();  // 采样 4000 次求陀螺零偏，每次 sleep 1 tick，约阻塞 4 s
                            // ！这几秒请让板子保持静止，否则 CALIBRATE_ERR 置位

        /* 初始化流程走完了。老师给的 ID 校验只在失败时置 INIT_ERR、成功时不清它，
         * 所以 INIT_ERR 会一直停在构造时的默认 true；由上层在流程成功后清掉。*/
        if (!g_imu.self_test.ACC_CHIP_ID_ERR && !g_imu.self_test.GYRO_CHIP_ID_ERR)
        {
            g_imu.self_test.INIT_ERR = false;
        }
    }

    for (;;)
    {
        g_imu.ReadAccData(&g_imu.acc_data);                    // m/s²
        g_imu.ReadAccTemperature(&g_imu.acc_data.temperature); // ℃，填进 acc_data 的温度字段
        g_imu.ReadGyroData(&g_imu.gyro_data);                  // rad/s
        g_imu.VerifyAccData();
        g_imu.VerifyGyroData();

#if IMU_SELF_TEST_LED
        ImuLedUpdate();
#endif

        tx_thread_sleep(IMU_THREAD_TICK);
    }
}
