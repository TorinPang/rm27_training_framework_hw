#include "threads.hpp"

#include "BMI088.hpp"
#include "main.h" //< LED_R/G/B 引脚宏（LED 是 README 允许线程直接调 HAL GPIO 的例外）

/* ==================================================================== *
 * 方向 C · imu 线程（TODO #10 的 imu 部分）
 *
 * 职责（最终形态）：BMI088 初始化、读三轴/温度，并把数据共享给其它线程。
 *
 * C1 阶段只做**读通路**的验证：不动任何寄存器，直接读两只芯片的 ID 并用三色灯表达结果。
 * 这样一旦不过，就能确定是 SPI 模式/引脚/片选的问题，与写通路无关，便于定位；
 * 数据解析（C2）、Config()/Calibrate() 与数据共享留到下一步再往这个线程里加。
 * ==================================================================== */

/* ---------------- 上板调试开关（风格同 main.c 的 BSP_SELF_TEST） ----------------
 * 1 = 用三色灯显示芯片 ID 校验结果：绿 = ACC/GYRO 都读到 0x1E/0x0F；红 = 至少一个不对。
 * 0 = 不驱动灯（方向 E 做出独立 LED 模块后，这段应删掉换成 LED 模块）*/
#define IMU_SELF_TEST_LED 1

/* ---------------- 线程参数 ---------------- */
#define IMU_THREAD_STACK_SIZE 1024U
#define IMU_THREAD_PRIORITY   10U   /* 比 control(8) 低：IMU 晚一拍不影响电机安全 */
#define IMU_THREAD_TICK       100U  /* 1 tick = 1 ms；C1 阶段循环里暂时没事做 */

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

static void ImuThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    /* 先给芯片和总线一点稳定时间（老师给的 Config() 也是这么开头的）。
     * 注意这几个读函数内部有 tx_thread_sleep，只能在内核启动之后（即线程里）调用。*/
    tx_thread_sleep(10);

    /* 热身读：重置后**第一次** SPI 事务实测出现过一次无效读（返回流里没有 0x1E），
     * 之后每次访问都正常。故这里先做一次丢弃读，把"第一次"消耗掉。*/
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

#if IMU_SELF_TEST_LED
    {
        const bool id_ok = (!g_imu.self_test.ACC_CHIP_ID_ERR) && (!g_imu.self_test.GYRO_CHIP_ID_ERR);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, id_ok ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, id_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    }
#endif

    for (;;)
    {
        tx_thread_sleep(IMU_THREAD_TICK);
    }
}
