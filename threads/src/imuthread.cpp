#include "threads.hpp"

#include "BMI088.hpp"

/* ==================================================================== *
 * 方向 C · imu 线程（TODO #10 的 imu 部分）
 *
 * 职责：BMI088 初始化（芯片 ID 自检 → Config → Calibrate），随后按周期读三轴
 *       加速度/角速度/温度并做通信自检；数据放在 g_imu 里供其它线程读。
 *
 * 现状：C1（读通路 + ID 自检）、C2（数据解析 + 单位换算 + 通信自检）已完成。
 *       灯效已交给方向 E 的 alive 线程（本线程不再直接驱动 GPIO）——
 *       本线程只负责"读数据"和"按周期上报存活"两件事：
 *         · 每轮 Alive_NotifyImuAlive()  → 表示 imu 线程还活着；
 *         · 初始化成功后 Alive_NotifyInitOk() → 表示系统初始化完成。
 *       数据共享仍走单写者约定（本线程写、其它线程读，不加互斥）。
 * ==================================================================== */

/* ---------------- 温控（方向 D · D2） ----------------
 * TEMP_CTRL_ENABLE  1 = 按 TEMP_CTRL_TARGET_C 闭环加热；0 = 恒不加热。
 *   加热电阻就在 BMI088 旁边，**板载即可验证**：设一个比室温高一截的目标，
 *   看温度往上走、duty 先大后小地收敛。首次上板建议先设 0 确认"不加热"这一侧。
 * TEMP_CTRL_TARGET_C 目标温度（℃）。室温约 25 ℃，取 35~40 既能看出明显升温、
 *   又不至于烤板子；设成低于当前温度（如 20）可作为"应完全不加热"的反向用例。*/
#define TEMP_CTRL_ENABLE   1
#define TEMP_CTRL_TARGET_C 40.0f

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

            /* 向 alive 线程上报"系统初始化完成"（方向 E）。这是一次性事件，
             * 报早了等于说谎，所以放在 Config/Calibrate 都跑完、INIT_ERR 清掉之后。
             * 若这里始终没执行（ID 错 / 初始化卡住），alive 线程会在 10 s 后判定"初始化失败"。*/
            Alive_NotifyInitOk();
        }
    }

    for (;;)
    {
        g_imu.ReadAccData(&g_imu.acc_data);                    // m/s²
        g_imu.ReadAccTemperature(&g_imu.acc_data.temperature); // ℃，填进 acc_data 的温度字段
        g_imu.ReadGyroData(&g_imu.gyro_data);                  // rad/s
        g_imu.VerifyAccData();
        g_imu.VerifyGyroData();

#if TEMP_CTRL_ENABLE
        /* 温控闭环（方向 D · D2）。放在读温度之后，保证用的是本拍刚采到的温度；
         * 也放在 Verify*Data 之后，让"加速度计通信失效"这一条能立刻把加热关掉。
         * 调用频率 = 线程周期 1 ms。热时间常数是"秒"级，1 kHz 远快于对象，对纯 P 无影响；
         * 将来 TempPid 若整定出积分项，需按 1 ms 的 dt 标定 ki（或改成每 N 拍调一次）。*/
        g_imu.TemperatureControl(TEMP_CTRL_TARGET_C);
#endif

        /* 向 alive 线程上报"imu 线程存活"（方向 E）：每轮一次，alive 线程据此判断
         * 本线程是否还在跑。注意上报点在**读数据之后** —— 只要线程还在转就上报，
         * 与"IMU 数据是否有效"无关（数据有效性由 self_test 标志表达，另行观察）。*/
        Alive_NotifyImuAlive();

        tx_thread_sleep(IMU_THREAD_TICK);
    }
}
