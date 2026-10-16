#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 静态成员的存储定义（声明在 DJIMotorHandler.hpp）。零初始化，故不需要显式初值。
 * 做成静态成员而非普通成员，是为了让它在符号表里有独立符号，
 * 上板时可直接观察报文打包结果（见头文件里的说明）。 */
uint8_t DJIMotorHandler::can1_send_data_0[8];
uint8_t DJIMotorHandler::can1_send_data_1[8];
uint8_t DJIMotorHandler::can2_send_data_0[8];
uint8_t DJIMotorHandler::can2_send_data_1[8];

/* ------------------------------------------------------------------ *
 * 协议常数（对照 DJI 手册与课程参考工程核对）
 * ------------------------------------------------------------------ */

/** @brief 编码器一圈的计数（13 位有效：0~8191，共 8192 个刻度） */
static constexpr float ENCODER_CNT = 8192.0f;
/** @brief 半圈计数，用于判定编码器读数是否发生回绕 */
static constexpr int ENCODER_HALF = 4096;
/** @brief 2π */
static constexpr float TWO_PI = 6.28318530718f;

/**
 * @brief 取减速箱的减速比（转子 -> 输出轴）。
 * @note 派生电机的构造函数里设置 gearBox，这里据此换算。
 */
static float GearRatioOf(GearBox gearBox)
{
    switch (gearBox)
    {
    case GearBox_M2006:
        return 36.0f;                   /*!< M2006 减速箱 36:1 */
    case GearBox_M3508:
        return 3591.0f / 187.0f;        /*!< M3508 减速箱 3591/187 ≈ 19.2032 */
    case GearBox_XRoll:
        return 268.0f / 17.0f;          /*!< 参考工程口径，待手册复核 */
    case GearBox_None:
    default:
        return 1.0f;
    }
}

/**
 * @brief 把一组 4 个电机槽位的电流按大端序打包进 8 字节控制帧。
 * @param slots 指向 4 个电机槽位（元素可为 nullptr，未注册的填 0）
 * @param buf   输出缓冲区（8 字节）
 * @note DJI 电机控制帧为大端：高字节在前。未注册的槽位写 0，避免输出随机电流。
 */
static void DJIMotorHandler_PackCurrents(DJIMotor *const *slots, uint8_t *buf)
{
    for (int i = 0; i < 4; ++i)
    {
        const int16_t cur = (slots[i] != nullptr) ? slots[i]->currentSet : 0;
        const uint16_t raw = static_cast<uint16_t>(cur);
        buf[2 * i]     = static_cast<uint8_t>((raw >> 8) & 0xFFU);
        buf[2 * i + 1] = static_cast<uint8_t>(raw & 0xFFU);
    }
}

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // 只登记 DJI 电机反馈/控制 ID 段，避免误接其它设备
    if ((motor == nullptr) || (canId < 0x201U) || (canId > 0x208U))
    {
        return;
    }

    uint8_t bus;
    if (hcan == &hcan1)
    {
        bus = 0;
    }
    else if (hcan == &hcan2)
    {
        bus = 1;
    }
    else
    {
        return;
    }

    const int index = static_cast<int>(canId - 0x201U); // 0~7
    DJIMotorList[bus][index] = motor;
    motor->hcan  = hcan;
    motor->canId = canId;

    // 0x201~0x204 由 0x200 报文控制，0x205~0x208 由 0x1FF 报文控制；
    // 置存在标志后 sendControlData 才会发对应报文（未注册的不发，防止堵塞总线）
    if (index < 4)
    {
        if (bus == 0) { CAN1_0x200_Exist = true; } else { CAN2_0x200_Exist = true; }
    }
    else
    {
        if (bus == 0) { CAN1_0x1FF_Exist = true; } else { CAN2_0x1FF_Exist = true; }
    }
}

void DJIMotorHandler::sendControlData()
{
    // 每条总线两组报文：0x200 承载索引 0~3，0x1FF 承载索引 4~7
    if (CAN1_0x200_Exist)
    {
        DJIMotorHandler_PackCurrents(DJIMotorList[0], can1_send_data_0);
        CAN_Transmit(&hcan1, 0x200U, can1_send_data_0, 8U);
    }
    if (CAN1_0x1FF_Exist)
    {
        DJIMotorHandler_PackCurrents(DJIMotorList[0] + 4, can1_send_data_1);
        CAN_Transmit(&hcan1, 0x1FFU, can1_send_data_1, 8U);
    }
    if (CAN2_0x200_Exist)
    {
        DJIMotorHandler_PackCurrents(DJIMotorList[1], can2_send_data_0);
        CAN_Transmit(&hcan2, 0x200U, can2_send_data_0, 8U);
    }
    if (CAN2_0x1FF_Exist)
    {
        DJIMotorHandler_PackCurrents(DJIMotorList[1] + 4, can2_send_data_1);
        CAN_Transmit(&hcan2, 0x1FFU, can2_send_data_1, 8U);
    }
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if ((rx_data == nullptr) || (index < 0) || (index > 7))
    {
        return;
    }

    uint8_t bus;
    if (hcan == &hcan1)
    {
        bus = 0;
    }
    else if (hcan == &hcan2)
    {
        bus = 1;
    }
    else
    {
        return;
    }

    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr)
    {
        return; // 该 ID 未注册电机，直接忽略
    }
    UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    if ((motor == nullptr) || (can_data == nullptr))
    {
        return;
    }

    DJIMotor::MotorFeedBack &fb = motor->motorFeedback;

    // 收到一帧反馈即计一次存活，供 AliveCheck 判断在线状态
    motor->AliveFlag++;

    fb.last_ecd        = static_cast<int16_t>(fb.ecd);
    fb.lastSpeedFdb    = fb.speedFdb;
    fb.lastPositionFdb = fb.positionFdb;

    // DJI 电机反馈帧（8 字节，大端）：
    // [0:1] 编码器  [2:3] 转速(rpm)  [4:5] 电流  [6] 温度  [7] 保留
    const uint16_t raw_ecd   = static_cast<uint16_t>((static_cast<uint16_t>(can_data[0]) << 8) | can_data[1]);
    const uint16_t raw_speed = static_cast<uint16_t>((static_cast<uint16_t>(can_data[2]) << 8) | can_data[3]);
    const uint16_t raw_cur   = static_cast<uint16_t>((static_cast<uint16_t>(can_data[4]) << 8) | can_data[5]);

    fb.ecd            = raw_ecd;
    fb.speed_rpm      = static_cast<int16_t>(raw_speed);
    fb.currentFdb     = static_cast<float>(static_cast<int16_t>(raw_cur)); // 原始计数值，换算系数依型号
    fb.temperatureFdb = static_cast<float>(can_data[6]);

    const float ratio = GearRatioOf(motor->gearBox);

    // 转速：转子 rpm -> 输出轴 rad/s
    fb.speedFdb = fb.speed_rpm * (TWO_PI / 60.0f) / ratio;

    // 位置：先按 ±半圈判定编码器回绕，再累加成输出轴连续角度（多圈、不回绕跳变）
    int delta = static_cast<int>(fb.ecd) - static_cast<int>(fb.last_ecd);
    if (delta > ENCODER_HALF)
    {
        delta -= static_cast<int>(ENCODER_CNT);
    }
    else if (delta < -ENCODER_HALF)
    {
        delta += static_cast<int>(ENCODER_CNT);
    }
    fb.positionFdb += static_cast<float>(delta) * (TWO_PI / ENCODER_CNT) / ratio;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    for (int bus = 0; bus < 2; ++bus)
    {
        for (int i = 0; i < 8; ++i)
        {
            if (DJIMotorList[bus][i] != nullptr)
            {
                DJIMotorList[bus][i]->AliveCheck();
            }
        }
    }
}

/**
 * @brief 覆盖 bsp_can.cpp 中的弱符号：把总线上的标准帧 CAN 报文分发到对应电机。
 * @note DJI 电机反馈 ID 0x201~0x208 与电机索引 0~7 一一对应。
 *       ID→索引的映射属于电机协议，故放在本层而不是 bsp；bsp 只负责“取帧”。
 */
void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data)
{
    if ((data == nullptr) || (StdId < 0x201U) || (StdId > 0x208U))
    {
        return;
    }
    DJIMotorHandler::Instance()->updateFeedback(hcan, data, static_cast<int>(StdId - 0x201U));
}
