#include "M2006.hpp"
#include "math.hpp"

/* ------------------------------------------------------------------ *
 * M2006 + C610 参数（对照 RoboMaster 官方规格）
 * ------------------------------------------------------------------ */

/** @brief 电流指令量程：C610 可控电流 -10000~10000，对应 -10A~10A */
static constexpr int M2006_MAX_CURRENT = 10000;

/** @brief 位置环输出（速度给定）限幅，单位 rad/s —— 双环里最重要的安全旋钮。
 *         **2026-10-15 上板整定值取 6**（三角波 ±6 rad 跟随良好、电流不顶限幅、角点削峰可接受）；
 *         官方空载 500 rpm ≈ 52.36 rad/s，调大即"更快更冲"（削峰变小、电流变大）。
 *         兼作位置环积分项限幅（`iResult` 本身就是速度给定，超过输出上限无意义）。 */
static constexpr float M2006_POS_SPEED_LIMIT = 6.0f;

/** @brief 速度环积分项限幅：约为电流量程的 30%，防止积分饱和 */
static constexpr float M2006_SPEED_I_LIMIT = 3000.0f;

/** @brief 离线判定去抖：连续 N 拍没收到新反馈才判 OFFLINE（1 拍 = 1 ms）。
 *         反馈与控制线程同为 1 kHz 但不同晶振 → 相位漂移会周期性出现单拍空窗，
 *         单拍判离线会误清 PID（积分被周期性抹掉）。取 5：真实拔线 5 ms 内仍能检出。 */
static constexpr uint32_t M2006_OFFLINE_CYCLES = 5U;

M2006::M2006()
{
    // 减速箱型号：反馈换算（转速、位置）依赖它，M2006 为 36:1
    gearBox = GearBox_M2006;

    // C610 电流指令量程 ±10000（±10A）
    maxCurrent = M2006_MAX_CURRENT;

    // 速度环：输出为电流指令，故输出/积分限幅以电流量程为界
    speedPid.maxOut  = static_cast<float>(maxCurrent);
    speedPid.maxIOut = M2006_SPEED_I_LIMIT;

    // 位置环：输出为速度给定(rad/s)，限幅以输出轴转速为界
    positionPid.maxOut  = M2006_POS_SPEED_LIMIT;
    positionPid.maxIOut = M2006_POS_SPEED_LIMIT;

    // 增益在基类（DJIMotor.hpp）给出（2026-10-15 上板整定值），这里只绑定与电机相关的限幅
    speedPid.Clear();
    positionPid.Clear();

    controlMode = RELAX_MODE;
    currentSet  = 0;
}

void M2006::setOutput()
{
    // 未收到反馈（离线）时不输出电流：否则反馈丢失后速度环会把误差积到饱和，电机失控。
    if (MotorState != MOTOR_ONLINE)
    {
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        return;
    }

    switch (controlMode)
    {
    case RELAX_MODE:
        // 松开：清空 PID 状态并输出 0 电流
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        break;

    case SPD_MODE:
        // 单速度环：速度误差 -> 电流指令
        speedPid.ref = speedSet;               // 目标输出轴角速度 rad/s
        speedPid.fdb = motorFeedback.speedFdb; // 实测输出轴角速度 rad/s
        speedPid.UpdateResult();
        currentSet = static_cast<int16_t>(
            Numeric::LimitABS(speedPid.result, static_cast<float>(maxCurrent)));
        break;

    case POS_MODE:
        // 双环串级：位置环输出(rad/s) 作为速度环给定
        positionPid.ref = positionSet;               // 目标角度 rad
        positionPid.fdb = motorFeedback.positionFdb; // 实测角度 rad
        positionPid.UpdateResult();                  // 结果已由 positionPid.maxOut 限幅

        speedPid.ref = positionPid.result;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = static_cast<int16_t>(
            Numeric::LimitABS(speedPid.result, static_cast<float>(maxCurrent)));
        break;

    default:
        currentSet = 0;
        break;
    }
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // 本周期内收到过新反馈（AliveFlag 被递增）即在线，否则离线。
    // 注意：不能“单拍没收到就判离线”——反馈与控制同为 1 kHz 但不同源，
    // 相位漂移会周期性出现单拍空窗，若直接判 OFFLINE，setOutput 会清 PID，
    // 积分被周期性抹掉（表现为：加 ki 消不掉静差、电流指令周期性掉 0）。
    if (AliveFlag == Pre_AliveFlag)
    {
        AliveMissCount++;
        if (AliveMissCount >= M2006_OFFLINE_CYCLES)
        {
            MotorState = MOTOR_OFFLINE;
        }
        // 未满 N 拍：维持 ONLINE，让积分继续积累
    }
    else
    {
        AliveMissCount = 0;
        MotorState = MOTOR_ONLINE;
    }
    Pre_AliveFlag = AliveFlag; // 无论在线与否都记录，供下个周期比较
    return MotorState;
}
