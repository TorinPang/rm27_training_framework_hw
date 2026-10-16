#include "threads.hpp"

#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "ref.hpp"
#include "can.h"

/* ==================================================================== *
 * 方向 B · control 线程（TODO #10）
 *
 * 职责：周期性跑电机控制 —— 更新在线状态 -> 计算电流 -> 组帧发出 CAN 报文。
 * 周期 1 ms（tx_user.h: TX_TIMER_TICKS_PER_SECOND = 1000，故 1 tick = 1 ms）。
 * ==================================================================== */

/* ---------------- 上板调试开关 ----------------
 * 与 main.c 的 BSP_SELF_TEST 同一风格：改这里、重新编译即可切换"上电默认状态"。
 *   0 = RELAX  上电即松开（输出电流恒 0）—— **安全默认**，也是"软件停机"的落点
 *   1 = SPD    上电即按 CONTROL_TEST_SPEED 转（调试期专用，记得用完改回 0）
 *   2 = POS    位置 + 速度双环（注意 positionFdb 无绝对零点，需先手动归零）
 * **整定期建议保持 0**：需要让电机转时用 PnX Platform 在线写 `control_tune_mode = 1`（或 2 走双环）
 * （在线改的是 RAM，掉电/复位即回到 0），这样复位/重新上电永远是静止的。
 * 正式框架里这两个量应由上层命令/遥控改写，这里只是临时给定值。 */
#define CONTROL_TEST_MODE  0
#define CONTROL_TEST_SPEED 10.0f /* rad/s（输出轴） */
#define CONTROL_TEST_POS   1.0f /* rad（输出轴） */

/* 电机在 CAN1 上的反馈/控制 ID（= 电调 ID 1，反馈 0x201 / 由 0x200 控制）。
 * 电调真实 ID 用 RoboMaster Assistant 设定；若不是 ID1，改这个数即可。 */
#define CONTROL_MOTOR_CAN_ID 0x201U

/* ---------------- 在线整定接口（供 PnX Platform 写变量） ----------------
 * 为什么需要这组"影子变量"：`control_motor` 的类型是 M2006，而 M2006 **一个自己的数据成员都没有**
 * —— `controlMode` / `speedSet` / `speedPid` 全部继承自基类 `DJIMotor`。
 * PnX Platform 的成员枚举只列**直接成员**，所以 `control_motor` 展开后是空的
 * （gdb / cortex-debug 支持继承，所以 VS Code 那边能看到 `control_motor.speedSet`，PnX 这边看不到）。
 * 对策：把它们镜像成**显式初始化的普通全局标量** —— 必进 PnX 的白名单，符号名稳定、无继承问题，
 * 整定时直接写这几个量，无需重编。
 * 置 CONTROL_TUNE_LIVE=0 时，下面的**变量定义**与线程里的**抄写/回读代码**全部不参与编译
 * （零 RAM、零 FLASH、零 CPU 开销），固件回到"纯框架"形态。
 *
 * **2026-10-15 晚两环整定完毕**（速度环 160/0.15/0、位置环 40/0.15/0 + 输出限幅 6 rad/s，
 * 均已写回 DJIMotor.hpp / M2006.cpp 的默认值），故**置 0 出干净固件**。
 * 以后再要在线整定：改回 1 → 重新编译 → 烧写，PnX 里这几个变量会重新出现。 */
#define CONTROL_TUNE_LIVE 0

#if CONTROL_TUNE_LIVE

/* 写入方向：整定给定与增益（控制线程每轮抄进电机对象） */
uint32_t control_tune_mode      = CONTROL_TEST_MODE;  /* 0=RELAX 1=SPD 2=POS；非法值落到 switch 的 default → 输出 0，安全 */
float    control_tune_speed_set = CONTROL_TEST_SPEED; /* rad/s（输出轴） */
float    control_tune_pos_set   = CONTROL_TEST_POS;   /* rad（输出轴） */
/* 速度环增益：2026-10-15 晚空载整定结果（与 DJIMotor.hpp:74 的默认值一致） */
float    control_tune_kp        = 160.0f;
float    control_tune_ki        = 0.15f;
float    control_tune_kd        = 0.0f;

/* 位置环（双环外环）整定量 —— 与 `DJIMotor.hpp` 的位置环默认值、`M2006.cpp` 的
 * `M2006_POS_SPEED_LIMIT` 保持一致（2026-10-15 上板整定结果）。
 * 量纲：输入 = 输出轴角度 rad，输出 = 速度给定 rad/s（作为内环 ref）。
 * max_spd 就是外环输出限幅（= 位置环能要求的最高转速），是双环里最重要的安全旋钮。 */
float    control_tune_pos_kp      = 40.0f;
float    control_tune_pos_ki      = 0.15f;
float    control_tune_pos_kd      = 0.0f;
float    control_tune_pos_max_spd = 6.0f;  /* rad/s；对应 M2006.cpp 的 M2006_POS_SPEED_LIMIT */
/* 零点对齐（数值变化沿触发）：写一个与当前值不同的数（1、2、3…）→ 把当前位置当作 0 rad，
 * 同时清位置环并把位置给定归零，避免"归零瞬间外环看到一个大误差"而窜车。
 * （编码器无绝对零点，进 POS 模式前必须先对齐；见线程循环里的使用处） */
uint32_t control_tune_pos_zero    = 0U;

/* 参考曲线（给定信号发生器）—— 整定用，语义见 libs/include/ref.hpp。
 * 单位跟随当前模式：SPD 模式是 rad/s、POS 模式是 rad。
 * shape=0(Constant) 时不用曲线，改用各模式自己的手动 set 变量（speed_set / pos_set）。 */
uint32_t control_tune_ref_shape     = 2U;     /* 0=常数 1=阶跃 2=三角 3=周期平滑阶跃 */
float    control_tune_ref_low       = -6.0f;  /* 曲线低值（三角/平滑阶跃的下限） */
float    control_tune_ref_high      = 6.0f;   /* 曲线高值（三角/平滑阶跃的上限） */
uint32_t control_tune_ref_step_ms   = 1000U;  /* Step：多少 ms 后跳到高值 */
uint32_t control_tune_ref_period_ms = 4000U;  /* Triangle / SmoothStep：周期 ms */

/* 回读方向：反馈镜像（只读，用于 PnX 的监看/实时曲线）。
 * 与上面同样的理由：`motorFeedback` 也在基类里，PnX 枚举不到。 */
float    control_fdb_speed    = 0.0f; /* rad/s（输出轴） */
float    control_fdb_position = 0.0f; /* rad（输出轴，多圈累计） */
float    control_fdb_current  = 0.0f; /* 电流反馈（原始计数） */
float    control_fdb_temp     = 0.0f; /* ℃ */
uint32_t control_fdb_ecd      = 0U;   /* 编码器原始读数 0~8191 */
uint32_t control_state        = 0U;   /* MotorState：0=OFFLINE 1=ONLINE（判在线用） */
float    control_cmd_current  = 0.0f; /* 本拍实际下发的电流指令（限幅后） */
/* 本拍实际目标值（= 曲线输出，或 Constant 模式下手动 set 值）。
 * 单位跟随模式：SPD → rad/s，POS → rad。画“给定 vs 反馈”时用它当给定曲线。 */
float    control_fdb_ref      = 0.0f;
/* 双环诊断：内环（速度环）的**给定** = 位置环的输出 rad/s。
 * 画 “control_fdb_ref（位置给定 rad）vs control_fdb_position（位置反馈 rad）”
 * 只看外环跟踪；画 “control_fdb_speed_ref vs control_fdb_speed” 看内环有没有跟上外环的要求。
 * 外环调不动时，先看这条线是不是被 max_spd 削平了。非 POS 模式时它保持上一次的值（无意义）。 */
float    control_fdb_speed_ref = 0.0f;
/* 累计“本拍没等到新反馈”的次数（AliveCheck 的空窗计数）。
 * 正常接入电调时它应缓慢增加（1 kHz 对 1 kHz 的相位漂移，约每秒零点几次）；
 * 若它暴涨，说明 CAN 反馈链路有问题。 */
uint32_t control_fdb_alive_miss = 0U;
#endif /* CONTROL_TUNE_LIVE */

/* ---------------- 线程参数 ---------------- */
#define CONTROL_THREAD_STACK_SIZE 1024U
#define CONTROL_THREAD_PRIORITY   8U    /* ThreadX：数值越小优先级越高 */
#define CONTROL_THREAD_TICK       1U    /* 1 tick = 1 ms */

static TX_THREAD control_thread;
/* ThreadX 要求线程栈 8 字节对齐，用属性显式保证（UCHAR 数组默认只保证 1 字节） */
static UCHAR control_thread_stack[CONTROL_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/* 电机对象放静态存储：生命周期必须覆盖整个运行期，且不能放在线程栈上。
 * `{}` 显式初始化是照 PnX Platform 写入白名单的要求写的（`T x{}` 形式），保留着无害。*/
static M2006 control_motor{};

#if CONTROL_TUNE_LIVE
/* 参考曲线实例 + 计时起点（ThreadX tick，1 tick = 1 ms）。
 * 曲线参数每拍从上面的 control_tune_ref_* 拷进来（见线程循环），所以在线改参数立刻生效。 */
static RefCurve control_ref_curve{};
static uint32_t control_ref_start_ms = 0U;

/* 零点对齐命令的"已应用值"（数值变化沿检测用；不是 PnX 接口，故为文件内 static） */
static uint32_t control_pos_zero_applied = 0U;

/** @brief 把在线写入的 uint32 形状码映射成枚举；未知取值落到 Constant（安全：给定不变）。 */
static RefShape ShapeFromCode(uint32_t code)
{
    switch (code)
    {
    case 1U:
        return RefShape::Step;
    case 2U:
        return RefShape::Triangle;
    case 3U:
        return RefShape::PeriodicSmoothStep;
    default:
        return RefShape::Constant;
    }
}
#endif /* CONTROL_TUNE_LIVE */

static void ControlThread_Entry(ULONG thread_input);

void ControlThread_Init(void)
{
    DJIMotorHandler *handler = DJIMotorHandler::Instance();

    // 绑定硬件：CAN1 + 电调 ID1。registerMotor 会置 CAN1_0x200_Exist，
    // 此后 sendControlData 才会真正发 0x200（未注册则不占总线）。
    handler->registerMotor(&control_motor, &hcan1, CONTROL_MOTOR_CAN_ID);

    // 上板调试用的模式与目标值（初始 RELAX：输出电流恒为 0，最安全）。
    // 注意：线程每轮都会用上面的 control_tune_* 覆盖这几个字段，这里的赋值只是"种子"，
    // 保证 CONTROL_TUNE_LIVE=0 时行为与过去完全一致。
    control_motor.controlMode = static_cast<DJIMotor::MotorControlModeType>(CONTROL_TEST_MODE);
    control_motor.speedSet    = CONTROL_TEST_SPEED;
    control_motor.positionSet = CONTROL_TEST_POS;

    (void)tx_thread_create(&control_thread,
                           (CHAR *)"control",
                           ControlThread_Entry,
                           0,
                           control_thread_stack,
                           CONTROL_THREAD_STACK_SIZE,
                           CONTROL_THREAD_PRIORITY,
                           CONTROL_THREAD_PRIORITY, /* 抢占门限 = 自身优先级 */
                           TX_NO_TIME_SLICE,
                           TX_AUTO_START);
}

static void ControlThread_Entry(ULONG thread_input)
{
    (void)thread_input;
    DJIMotorHandler *handler = DJIMotorHandler::Instance();

#if CONTROL_TUNE_LIVE
    /* 曲线计时起点 = 线程启动时刻（曲线的 t=0 在这里，而不是上电/复位时刻） */
    control_ref_start_ms = tx_time_get();
#endif

    for (;;)
    {
        // 1) 先用上一周期收到的反馈更新在线状态（离线时 setOutput 会自动输出 0 并清 PID）
        handler->AllMotorAliveCheck();

        // 1.5) 在线整定：把影子变量抄进电机对象（同一拍写入、同一拍生效）
#if CONTROL_TUNE_LIVE
        /* 参考曲线：参数每拍刷新（在线改立即生效），再取当前时刻的给定值。
         * Constant 形状不用曲线，改用手动 set 变量 —— 与参考工程一致：
         * 实验顺序 Constant（确认方向）→ Step（阶跃）→ Triangle（跟随）。 */
        control_ref_curve.shape    = ShapeFromCode(control_tune_ref_shape);
        control_ref_curve.low      = control_tune_ref_low;
        control_ref_curve.high     = control_tune_ref_high;
        control_ref_curve.delayMs  = control_tune_ref_step_ms;
        control_ref_curve.periodMs = control_tune_ref_period_ms;

        const bool  refFromCurve = (control_ref_curve.shape != RefShape::Constant);
        const float curveValue   = refFromCurve
                                       ? control_ref_curve.Value(tx_time_get() - control_ref_start_ms)
                                       : 0.0f;

        /* 零点对齐（只对 POS 双环有意义）：编码器是增量式的、没有绝对零点，
         * 上电后 positionFdb 是从"上电那一点"开始累加的。所以在进位置环之前，
         * 必须先把手当前姿态定义成 0 rad —— 否则写个 positionSet=1，电机可能直接甩 1 rad 找目标。
         * 归零同时清位置环、把给定归零，避免"归零瞬间外环看到一个大误差"而窜车。
         *
         * 触发方式 = **数值变化沿**（不是"写 1 就清"）：写一个与当前值不同的数（1、2、3…）触发一次。
         * 这样变量保持在被写的值上、不自我清零 —— 否则 PnX 写完回读时读到 0 会报
         * `Target readback did not verify`（写→回读之间有 1 ms，线程已经把它改回去了）。
         * 想再对齐一次就换个新数值再写。 */
        if (control_tune_pos_zero != control_pos_zero_applied)
        {
            control_pos_zero_applied = control_tune_pos_zero;
            control_motor.motorFeedback.positionFdb     = 0.0f;
            control_motor.motorFeedback.lastPositionFdb = 0.0f;
            control_motor.positionPid.Clear();
            control_motor.positionSet = 0.0f;
        }

        control_motor.controlMode = static_cast<DJIMotor::MotorControlModeType>(control_tune_mode);
        control_motor.speedSet    = refFromCurve ? curveValue : control_tune_speed_set;
        control_motor.positionSet = refFromCurve ? curveValue : control_tune_pos_set;
        control_motor.speedPid.kp = control_tune_kp;
        control_motor.speedPid.ki = control_tune_ki;
        control_motor.speedPid.kd = control_tune_kd;
        /* 外环（位置环）：输出限幅 = 允许的最高转速给定，是双环里最重要的安全旋钮 */
        control_motor.positionPid.kp      = control_tune_pos_kp;
        control_motor.positionPid.ki      = control_tune_pos_ki;
        control_motor.positionPid.kd      = control_tune_pos_kd;
        control_motor.positionPid.maxOut  = control_tune_pos_max_spd;
        control_motor.positionPid.maxIOut = control_tune_pos_max_spd;
#endif

        // 2) 计算本周期电流（含双环 PID 与限幅）
        control_motor.setOutput();

        // 2.5) 回读镜像：把反馈抄到全局标量，供 PnX 监看/画曲线（只读方向）
#if CONTROL_TUNE_LIVE
        control_fdb_speed    = control_motor.motorFeedback.speedFdb;
        control_fdb_position = control_motor.motorFeedback.positionFdb;
        control_fdb_current  = control_motor.motorFeedback.currentFdb;
        control_fdb_temp     = control_motor.motorFeedback.temperatureFdb;
        control_fdb_ecd      = control_motor.motorFeedback.ecd;
        control_state        = static_cast<uint32_t>(control_motor.MotorState);
        control_cmd_current  = static_cast<float>(control_motor.currentSet);
        control_fdb_alive_miss = control_motor.AliveMissCount;
        /* 双环诊断：位置环输出（= 内环给定 rad/s）。仅在 POS 模式下有意义。 */
        control_fdb_speed_ref = control_motor.positionPid.result;
        /* 实际给定：POS 模式取位置目标(rad)，其余取速度目标(rad/s) */
        control_fdb_ref = (control_motor.controlMode == DJIMotor::POS_MODE)
                              ? control_motor.positionSet
                              : control_motor.speedSet;
#endif

        // 3) 组帧并发出 0x200 / 0x1FF
        handler->sendControlData();

        // 4) 向 alive 线程上报"本线程存活"（方向 E）。报的是线程在跑，与电机是否在线无关：
        //    电机没接/掉线时本线程照样在跑、照样上报，此时该由 M2006::AliveCheck 判 OFFLINE。
        Alive_NotifyMotorAlive();

        tx_thread_sleep(CONTROL_THREAD_TICK);
    }
}
