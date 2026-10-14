#include "threads.hpp"

#include "bsp_led.hpp"

/* ==================================================================== *
 * 方向 E · alive 线程（TODO #10 的 alive 部分）+ LED 模块（TODO #11）
 *
 * 职责：用 3 个计数型信号量监控板子的三件事，并把状态映射成三色灯灯效。
 *       README 要求的三件事 —— 初始化是否成功、BMI088 线程是否存活、电机控制线程是否存活。
 *
 * 监控模型（关键：判据是"上报率"，不是"某一次的读数"）：
 *       · 被监控方**每个周期主动上报**（tx_semaphore_put）；
 *       · alive 线程**每个周期把信号量排空**（反复 tx_semaphore_get 到空），
 *         得到"这一周期内收到多少次上报"；
 *       · 次数低于阈值 ⇒ 该方已停止上报（线程死 / 卡住 / 没跑起来）。
 *
 *       为什么必须"排空"而不是只 get 一次：计数型信号量的计数会一直累加，
 *       只取一次的话，即便上报方早就停了，计数里还留着旧值 → 永远判不出"死了"。
 *
 * 为什么 alive 周期(100 ms) 远大于被监控线程周期(1 ms)：
 *       两者差 100 倍，留给调度抖动很大余量；阈值取 5 次即判"活着"，
 *       既不会误报（正常每周期约 100 次），也不会漏报（线程真死了必然是 0 次）。
 * ==================================================================== */

/* ---------------- 开关（§2：每个功能都能单独验证） ----------------
 * ALIVE_ENABLE     1 = alive 线程驱动灯效；0 = 只监控不点灯（排查灯效干扰时置 0）。
 * ALIVE_THREAD_*   线程参数：栈 512 B、优先级 20（全场最低，见 AGENTS.md §6）。
 * ALIVE_*_PERIODS  以 alive 周期为单位的窗口/节拍，避免直接写毫秒数。*/
#define ALIVE_ENABLE       1
#define ALIVE_STACK_SIZE   512U
#define ALIVE_PRIORITY     20U
#define ALIVE_PERIOD_TICKS 100U /* 100 ticks = 100 ms 一轮（tick = 1 ms） */

#define ALIVE_INIT_TIMEOUT_PERIODS 100U  /* 100 × 100 ms = 10 s 内没报"初始化成功" ⇒ 判定失败 */
#define ALIVE_MIN_REPORTS          5U    /* 一周期内上报少于 5 次 ⇒ 判定该线程已停 */
#define ALIVE_BLINK_PERIODS        5U    /* 每 5 轮(500 ms)翻转一次 ⇒ 1 Hz 慢闪 */
#define ALIVE_DRAIN_MAX            2000U /* 排空信号量的安全上限，防死循环 */

/* ---------------- 故障注入开关（专用于验收方向 E） ----------------
 * 验收要求"初始化失败 / imu 线程死 / control 线程死 三种灯效可区分"，故留三个开关逐条注入：
 * 置 1 时**照常排空信号量、但把这一路的计数丢弃**，效果等价于"该方没有上报"。
 * 一次性只置一个，重新编译上板看灯，就能把三种状态分别验一遍。
 * （三种都必须置回 0 才是正常固件。）*/
#define ALIVE_INJECT_INIT_FAIL  0 /* 1 = 假装初始化失败   ⇒ 10 s 后应为 红常亮 */
#define ALIVE_INJECT_IMU_DEAD   0 /* 1 = 假装 imu 线程死  ⇒ 应为 蓝常亮 */
#define ALIVE_INJECT_MOTOR_DEAD 0 /* 1 = 假装 control 死  ⇒ 应为 红蓝交替 */

/* 三个信号量（初始计数 0）+ alive 线程控制块与静态栈。
 * 栈必须 8 字节对齐（ThreadX 要求），UCHAR 数组默认只保证 1 字节，故显式加属性。*/
static TX_SEMAPHORE init_ok_sem;
static TX_SEMAPHORE imu_alive_sem;
static TX_SEMAPHORE motor_alive_sem;

static TX_THREAD alive_thread;
static UCHAR alive_thread_stack[ALIVE_STACK_SIZE] __attribute__((aligned(8)));

static void AliveThread_Entry(ULONG thread_input);

/* ---------------- 上报接口（给 control / imu 线程调用） ---------------- */

void Alive_NotifyInitOk(void)
{
    /* 一次性事件：初始化成功只报一次，alive 线程取到后 latch 住。*/
    (void)tx_semaphore_put(&init_ok_sem);
}

void Alive_NotifyImuAlive(void)
{
    (void)tx_semaphore_put(&imu_alive_sem);
}

void Alive_NotifyMotorAlive(void)
{
    /* 注意：这里报的是"control 线程还活着"，与"电机是否在线"无关 ——
     * 电机没接、掉线时线程照样在跑，照样上报。电机在线与否由 M2006::AliveCheck 判定，
     * 两者不要混用。*/
    (void)tx_semaphore_put(&motor_alive_sem);
}

/* ---------------- 初始化 ---------------- */

void AliveThread_Init(void)
{
    /* 信号量必须先于被监控线程创建：control / imu 线程一开始跑就会立即上报，
     * 信号量晚建会丢上报（甚至 put 到不存在的对象上报错）。
     * 本函数在 App_ThreadX_Init() 里被**第一个**调用，就是为了保证这个先后顺序。*/
    (void)tx_semaphore_create(&init_ok_sem, (CHAR *)"init_ok", 0);
    (void)tx_semaphore_create(&imu_alive_sem, (CHAR *)"imu_alive", 0);
    (void)tx_semaphore_create(&motor_alive_sem, (CHAR *)"motor_alive", 0);

    (void)tx_thread_create(&alive_thread,
                           (CHAR *)"alive",
                           AliveThread_Entry,
                           0,
                           alive_thread_stack,
                           ALIVE_STACK_SIZE,
                           ALIVE_PRIORITY,
                           ALIVE_PRIORITY, /* 抢占门限 = 自身优先级 */
                           TX_NO_TIME_SLICE,
                           TX_AUTO_START);
}

/* ---------------- 线程主体 ---------------- */

/* 把信号量排空并返回这一周期内取到的次数。
 * 上限 ALIVE_DRAIN_MAX 只是防御性的：正常一轮约 100 次，上限给到 2000
 * （相当于容忍 alive 被饿死 2 s），不会成为常规路径。*/
static UINT Alive_DrainReports(TX_SEMAPHORE *sem)
{
    UINT count = 0U;
    while ((count < ALIVE_DRAIN_MAX) && (tx_semaphore_get(sem, TX_NO_WAIT) == TX_SUCCESS))
    {
        ++count;
    }
    return count;
}

static void AliveThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    bool init_ok = false;  // latch：取到过一次 init_ok 就置 true，永不回退
    uint32_t periods = 0U; // 已运行周期数（初始化超时判定用）
    uint32_t blink_cnt = 0U;
    uint8_t blink_on = 1U;

    for (;;)
    {
        /* 1) 采集：把三个信号量排空，得到本周期各自的上报次数。
         *    注意"排空"这一步无论是否丢弃都必须做：不排空计数会一直累加，
         *    之后即便上报方停了，计数里还留着旧值，就永远判不出"死了"。*/
        UINT init_reports = Alive_DrainReports(&init_ok_sem);
        UINT imu_reports = Alive_DrainReports(&imu_alive_sem);
        UINT motor_reports = Alive_DrainReports(&motor_alive_sem);

        /* 故障注入（验收用，平时三个宏都是 0，编译器会把下面三行优化掉） */
        if (ALIVE_INJECT_INIT_FAIL)
        {
            init_reports = 0U;
        }
        if (ALIVE_INJECT_IMU_DEAD)
        {
            imu_reports = 0U;
        }
        if (ALIVE_INJECT_MOTOR_DEAD)
        {
            motor_reports = 0U;
        }

        /* 首次拿到 init_ok 的那一周期要"宽限"：imu 线程可能刚跑完初始化进入主循环，
         * 本周期内的上报次数还不足（窗口没被填满），直接参与存活判定会误报一次"线程死"。
         * 故这一周期只显示"正常"，从下一周期（已是一个完整 100 ms 窗口）才开始判存活。*/
        const bool init_ok_just_latched = (init_reports > 0U) && (!init_ok);
        if (init_reports > 0U)
        {
            init_ok = true;
        }

        /* 2) 闪烁节拍（1 Hz） */
        ++periods;
        if (++blink_cnt >= ALIVE_BLINK_PERIODS)
        {
            blink_cnt = 0U;
            blink_on = (blink_on != 0U) ? 0U : 1U;
        }

        /* 3) 判定 + 灯效。从上到下优先级递减，每个状态独占一种灯效，肉眼可区分：
         *
         *   初始化中     绿慢闪     上电后 init_ok 还没到（BMI088 初始化约 4.5 s）
         *   初始化失败   红常亮     ALIVE_INIT_TIMEOUT_PERIODS 内始终没等到 init_ok
         *   全部正常     绿常亮     三方都正常
         *   IMU 线程死   蓝常亮     imu 线程停止上报
         *   电机线程死   红蓝交替   control 线程停止上报
         *   两个都死     红+蓝常亮  与"初始化失败(纯红)"、"IMU死(纯蓝)"都能区分
         */
#if ALIVE_ENABLE
        if (!init_ok)
        {
            if (periods >= ALIVE_INIT_TIMEOUT_PERIODS)
            {
                LED_SetAll(1U, 0U, 0U); /* 初始化失败：红常亮 */
            }
            else
            {
                LED_SetAll(0U, blink_on, 0U); /* 初始化中：绿慢闪 */
            }
        }
        else
        {
            const bool imu_dead = (!init_ok_just_latched) && (imu_reports < ALIVE_MIN_REPORTS);
            const bool motor_dead = (!init_ok_just_latched) && (motor_reports < ALIVE_MIN_REPORTS);

            if (imu_dead && motor_dead)
            {
                LED_SetAll(1U, 0U, 1U); /* 两个线程都死：红+蓝常亮 */
            }
            else if (imu_dead)
            {
                LED_SetAll(0U, 0U, 1U); /* IMU 线程死：蓝常亮 */
            }
            else if (motor_dead)
            {
                LED_SetAll(blink_on, 0U, (blink_on != 0U) ? 0U : 1U); /* 电机线程死：红蓝交替 */
            }
            else
            {
                LED_SetAll(0U, 1U, 0U); /* 全部正常：绿常亮 */
            }
        }
#endif

        tx_thread_sleep(ALIVE_PERIOD_TICKS);
    }
}
