#ifndef DJIMOTOR_HPP
#define DJIMOTOR_HPP

#include "pid.hpp"
#include "can.h"

enum GearBox
{
    GearBox_None = 0,  ///< 无减速箱
    GearBox_M2006 = 1, ///< M2006减速箱
    GearBox_M3508 = 2, ///< M3508减速箱
    GearBox_XRoll = 3  ///< XRoll减速箱
};

/**
 * @class DJIMotor
 * @brief 电机控制类，提供电机的基本控制功能。
 *
 * 保留速度、位置控制所需的状态；控制计算由派生类完成。
 */
class DJIMotor
{
public:
    /**
     * @enum MotorControlModeType
     * @brief 描述电机的不同控制模式。
     */
    enum MotorControlModeType
    {
        RELAX_MODE = 0,         ///< 电机松开模式，所有输出均为0。
        SPD_MODE = 1,           ///< 速度模式，控制电机速度。
        POS_MODE = 2            ///< 位置模式，控制电机到特定位置。
    };

    enum MotorStateTypedef
    {
        MOTOR_OFFLINE = 0,
        MOTOR_ONLINE = 1
    };

    /**
     * @struct MotorFeedBack
     * @brief 电机反馈数据的结构体，包括电机的各种物理量反馈。
     * 结构体中包含了电机的电流、速度、位置等信息，以及电机的温度等状态反馈。
     */
    struct MotorFeedBack
    {
        int16_t last_ecd;      ///< 上次电机编码器的读数
        uint16_t ecd;          ///< 当前电机编码器的读数
        int16_t speed_rpm;     ///< 电机的转速，单位rpm
        float currentFdb;    ///< 电机电流反馈
        float speedFdb;        ///< 电机当前速度反馈, 单位rad/s
        float lastSpeedFdb;    ///< 上次记录的电机速度
        float positionFdb;     ///< 电机当前位置反馈
        float lastPositionFdb; ///< 上次记录的电机位置
        float temperatureFdb;  ///< 电机温度反馈
    };

    MotorStateTypedef MotorState;
    MotorControlModeType controlMode; ///< 当前电机控制模式
    MotorFeedBack motorFeedback;      ///< 处理后的电机的反馈数据
    uint16_t canId;                   ///< 电机的CAN通信ID
    CAN_HandleTypeDef *hcan;          ///< 指向电机使用的CAN接口的指针

    /* ---------------- 双环 PID（均为位置式） ----------------
     * 量纲约定：
     *   速度环：输入 = 输出轴角速度(rad/s)，输出 = 电流指令(原始计数，±10000 对应 ±10A)
     *   位置环：输入 = 输出轴角度(rad)，输出 = 速度给定(rad/s)，串级送入速度环
     * 增益为 M2006 起步值（先纯 P 保证安全），上板后按“先 P、后 I、再 D”整定。
     * maxOut / maxIOut 是与电机相关的量，由派生类构造函数按各自型号设置（见 M2006.cpp）：
     *   speedPid.maxOut    = maxCurrent     （电流指令上限）
     *   positionPid.maxOut = 输出轴转速上限（速度给定上限）
     * 此处保持 0 作安全默认：万一派生类忘记设置，输出为 0 而非失控。
     *
     * 【速度环整定结果 · 2026-10-15 晚间上板】空载 M2006，kp=160 / ki=0.15 / kd=0：
     * 常数段静差为零（纯 P 160 时为 8.45 vs 给定 10），阶跃无超调无持续振荡，
     * 三角波 ±6 rad/s 跟随良好（过零处因静摩擦有 stick-slip 平台，属硬件特性）。
     * 【位置环整定结果 · 同日】kp=40 / ki=0.15 / kd=0（输出限幅 6 rad/s，见 M2006.cpp）：
     * 三角波 ±6 rad 位置跟随良好，内环 `speed_ref` 全程方波（外环基本顶限幅），
     * 角点削峰 ~0.5~1 rad 属"内部限速 + 摩擦"的物理代价。 */
    PID speedPid = PID(160.0f, 0.15f, 0.0f, 0.0f, 0.0f, PID_POSITION);
    PID positionPid = PID(40.0f, 0.15f, 0.0f, 0.0f, 0.0f, PID_POSITION);

    float speedSet;    ///< 设定的目标速度
    float positionSet; ///< 设定的目标位置，范围[-Π, Π]

    int16_t currentSet;  ///< 设定的电流输出
    uint16_t maxCurrent; ///< 最大电流限制

    uint32_t AliveFlag;
    uint32_t Pre_AliveFlag;
    /* 连续“本拍没等到新反馈”的拍数（AliveCheck 去抖用）。
     * 反馈 1 kHz、控制线程也 1 kHz，两者不同晶振、频率有 ppm 级偏差 →
     * 相位缓慢漂移，周期性地出现“某一拍恰好没等到新帧”。
     * 若单拍未收到就判离线，会周期性误清 PID（积分被抹掉、电流指令瞬间归 0）。
     * 因此离线判定必须看“连续 N 拍”而不是单拍。 */
    uint32_t AliveMissCount;

    GearBox gearBox;
    /**
     * @brief 纯虚函数，用于设置电机输出。
     * 必须在派生类中实现此函数。
     */
    virtual void setOutput() = 0;

    virtual MotorStateTypedef AliveCheck() = 0;

    virtual ~DJIMotor() = default;

    /**
     * @brief 构造函数
     */
    DJIMotor()
    {
        controlMode = RELAX_MODE;

        speedSet = 0;
        positionSet = 0;
        currentSet = 0;

        maxCurrent = 0;

        motorFeedback = {};
        canId = 0;
        hcan = nullptr;
        AliveFlag = Pre_AliveFlag = 0;
        AliveMissCount = 0;
        gearBox = GearBox_None;
        MotorState = MOTOR_OFFLINE;
    }
};

#endif //DJIMOTOR_HPP
