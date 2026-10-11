#include "threads.hpp"

#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "can.h"

/* ==================================================================== *
 * 方向 B · control 线程（TODO #10）
 *
 * 职责：周期性跑电机控制 —— 更新在线状态 -> 计算电流 -> 组帧发出 CAN 报文。
 * 周期 1 ms（tx_user.h: TX_TIMER_TICKS_PER_SECOND = 1000，故 1 tick = 1 ms）。
 * ==================================================================== */

/* ---------------- 上板调试开关（步骤 4 验证用） ----------------
 * 与 main.c 的 BSP_SELF_TEST 同一风格：改这里、重新编译即可切换要验的东西。
 *   0 = RELAX  只发 0 电流，用来确认 0x200 报文 ID / 8 字节大端序 / 电流为 0
 *   1 = SPD    单速度环，看 speedFdb 是否跟随 speedSet（**需先整定 PID**）
 *   2 = POS    位置 + 速度双环（注意 positionFdb 无绝对零点，需先手动归零）
 * 正式框架里这两个量应由上层命令/遥控改写，这里只是临时给定值。 */
#define CONTROL_TEST_MODE  0
#define CONTROL_TEST_SPEED 5.0f /* rad/s（输出轴） */
#define CONTROL_TEST_POS   1.0f /* rad（输出轴） */

/* 电机在 CAN1 上的反馈/控制 ID（= 电调 ID 1，反馈 0x201 / 由 0x200 控制）。
 * 电调真实 ID 用 RoboMaster Assistant 设定；若不是 ID1，改这个数即可。 */
#define CONTROL_MOTOR_CAN_ID 0x201U

/* ---------------- 线程参数 ---------------- */
#define CONTROL_THREAD_STACK_SIZE 1024U
#define CONTROL_THREAD_PRIORITY   8U    /* ThreadX：数值越小优先级越高 */
#define CONTROL_THREAD_TICK       1U    /* 1 tick = 1 ms */

static TX_THREAD control_thread;
/* ThreadX 要求线程栈 8 字节对齐，用属性显式保证（UCHAR 数组默认只保证 1 字节） */
static UCHAR control_thread_stack[CONTROL_THREAD_STACK_SIZE] __attribute__((aligned(8)));

/* 电机对象放静态存储：生命周期必须覆盖整个运行期，且不能放在线程栈上 */
static M2006 control_motor;

static void ControlThread_Entry(ULONG thread_input);

void ControlThread_Init(void)
{
    DJIMotorHandler *handler = DJIMotorHandler::Instance();

    // 绑定硬件：CAN1 + 电调 ID1。registerMotor 会置 CAN1_0x200_Exist，
    // 此后 sendControlData 才会真正发 0x200（未注册则不占总线）。
    handler->registerMotor(&control_motor, &hcan1, CONTROL_MOTOR_CAN_ID);

    // 上板调试用的模式与目标值（初始 RELAX：输出电流恒为 0，最安全）
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

    for (;;)
    {
        // 1) 先用上一周期收到的反馈更新在线状态（离线时 setOutput 会自动输出 0 并清 PID）
        handler->AllMotorAliveCheck();
        // 2) 计算本周期电流（含双环 PID 与限幅）
        control_motor.setOutput();
        // 3) 组帧并发出 0x200 / 0x1FF
        handler->sendControlData();

        tx_thread_sleep(CONTROL_THREAD_TICK);
    }
}
