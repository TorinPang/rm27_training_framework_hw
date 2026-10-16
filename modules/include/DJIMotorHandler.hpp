#ifndef DJIMOTORHANDLER_HPP
#define DJIMOTORHANDLER_HPP

#include "DJIMotor.hpp"

/**
 * @brief 电机控制类
 */
class DJIMotorHandler
{
public:
    /**
     *@brief 使用指针数组, 2个CAN口，每个CAN口最多8个电机
     */
    DJIMotor *DJIMotorList[2][8]{};

    /* 发送缓冲区声明为 static：它们只承载"本拍要发出去的 8 字节"，不属于某个实例的状态
     * （本类本就是单例，语义不变），但做成静态成员后会有独立符号，
     * 上板时可直接在调试器里看 `DJIMotorHandler::can1_send_data_0` 来验证报文打包。
     * 定义在 DJIMotorHandler.cpp。 */
    static uint8_t can1_send_data_0[8]; // CAN1电机控制数据，用于控制0x201-0x204
    static uint8_t can1_send_data_1[8]; // CAN1电机控制数据，用于控制0x205-0x208

    static uint8_t can2_send_data_0[8]; // CAN2电机控制数据，用于控制0x201-0x204
    static uint8_t can2_send_data_1[8]; // CAN2电机控制数据，用于控制0x205-0x208

    // 四个标志位，用于判断是否需要发送控制数据
    bool CAN1_0x200_Exist = false; // CAN1是否存在控制报文为0x200电机
    bool CAN1_0x1FF_Exist = false; // CAN1是否存在控制报文为0x1FF电机
    bool CAN2_0x200_Exist = false; // CAN2是否存在控制报文为0x200电机
    bool CAN2_0x1FF_Exist = false; // CAN2是否存在控制报文为0x1FF电机

    /**
     * @brief 注册电机，将电机指针存入MotorList中
     * @param DJImotor 电机指针
     * @param hcan CAN句柄
     * @param canId 电机ID
     */
    void registerMotor(DJIMotor *DJImotor, CAN_HandleTypeDef *hcan, uint16_t canId); // 使用指针作为参数

    /**
     * @brief 发送控制数据
     */
    void sendControlData();

    /**
     * @brief 处理并更新电机反馈数据
     * @param hcan CAN 句柄
     * @param rx_data 反馈数据
     * @param index 电机索引
     */
    void updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index);

    void AllMotorAliveCheck();

    void UpdateSensorData(DJIMotor *motor, uint8_t *can_receive_data);

    static DJIMotorHandler *Instance()
    {
        static DJIMotorHandler instance;
        return &instance;
    }
};

#endif // DJIMOTORHANDLER_HPP
