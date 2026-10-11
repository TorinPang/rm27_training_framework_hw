#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include <cmath>

namespace BMI088
{
    namespace
    {
        /* 片选映射：把 cs 枚举翻成 CubeMX 生成的引脚宏（见 BMI088.hpp 的硬件连接一节）。
         * 两只从设备共用总线但片选独立，任何时刻只能有一个 CS 为低。*/
        GPIO_TypeDef *CsPort(enum BMI088_SENSOR cs)
        {
            return (cs == BMI088_CS_ACC) ? BMI088_ACC_CS_GPIO_PORT : BMI088_GYRO_CS_GPIO_PORT;
        }

        uint16_t CsPin(enum BMI088_SENSOR cs)
        {
            return (cs == BMI088_CS_ACC) ? BMI088_ACC_CS_PIN : BMI088_GYRO_CS_PIN;
        }

        /* 单次 SPI 事务的超时（ms）。5.25 Mbit/s 下 8 个字节不到 15 µs，10 ms 已很宽松。*/
        constexpr uint32_t BMI088_SPI_TIMEOUT = 10U;

        /* 单次传输的字节上限：加速度计 burst 读要 1(哑字节) + 6(三轴) = 7，取 8 留余量。*/
        constexpr uint8_t BMI088_SPI_MAX_LEN = 8U;
    }

    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        // TODO: 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        (void)target_temp;
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[1] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData[1] == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData()
    {
        // TODO: 检查加速度数据是否有效，并更新 ACC_DATA_ERR。
    }

    void cBMI088::VerifyGyroData()
    {
        // TODO: 检查角速度数据是否有效，并更新 GYRO_DATA_ERR。
    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0U || len > BMI088_SPI_MAX_LEN) return;

        /* 写协议：先发 1 字节地址（bit0=0 表示写、bit1~7 为寄存器地址），随后紧跟 len 字节数据。
         * 地址把最高位清掉 —— BMI088_SPI_WRITE_CODE(0x7F) 就是干这个的掩码。*/
        uint8_t tx_addr = static_cast<uint8_t>(addr & BMI088_SPI_WRITE_CODE);

        HAL_GPIO_WritePin(CsPort(cs), CsPin(cs), GPIO_PIN_RESET);
        if (HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &tx_addr, 1U, BMI088_SPI_TIMEOUT) == HAL_OK)
        {
            (void)HAL_SPI_Transmit(&BMI088_SPI_HANDLE, data, len, BMI088_SPI_TIMEOUT);
        }
        HAL_GPIO_WritePin(CsPort(cs), CsPin(cs), GPIO_PIN_SET);
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0U || len > BMI088_SPI_MAX_LEN) return;

        /* 读协议：先发 1 字节地址（bit7=1 表示读），之后的字节才是从机送回来的内容。
         * 关键差别（数据手册 §6.1.2 / §6.1.1）：
         *   - 加速度计在地址之后还会多送 1 个哑字节，第 2 个字节才是真数据
         *     （"单字节读"要按 burst 读 2 字节、丢弃第 1 个）；
         *   - 陀螺仪没有哑字节，地址之后当场就是数据。
         * 所以这里按原样把 len 字节回传给调用方、不替它丢弃 —— 这样已经写好的
         * VerifyAccChipID（读 2 取 [1]）与 VerifyGyroChipID（读 1 取 [0]）都能直接成立。*/
        uint8_t tx[BMI088_SPI_MAX_LEN + 1U];
        uint8_t rx[BMI088_SPI_MAX_LEN + 1U];

        tx[0] = static_cast<uint8_t>(addr | BMI088_SPI_READ_CODE);
        for (uint8_t i = 1U; i <= len; ++i) tx[i] = 0xFFU; // 读的时候 MOSI 上送什么是无所谓的

        HAL_GPIO_WritePin(CsPort(cs), CsPin(cs), GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&BMI088_SPI_HANDLE,
                                                           tx, rx,
                                                           static_cast<uint16_t>(len) + 1U,
                                                           BMI088_SPI_TIMEOUT);
        HAL_GPIO_WritePin(CsPort(cs), CsPin(cs), GPIO_PIN_SET);

        if (status == HAL_OK)
        {
            for (uint8_t i = 0U; i < len; ++i) data[i] = rx[i + 1U]; // rx[0] 是发地址期间的无效字节
        }
        else
        {
            for (uint8_t i = 0U; i < len; ++i) data[i] = 0U; // 通信失败清零，避免上级读到未初始化数据
        }
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        // TODO: 按 BMI088 加速度计 SPI 协议丢弃首个无效字节，拼接三轴有符号原始值，
        // 再按配置的量程换算为 m/s²，写入 data。
        if (data != nullptr) *data = {};
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        // TODO: 读取并拼接陀螺仪三轴原始值，按配置量程换算为 rad/s，
        // 减去 Gyro_offset 后写入 data。
        if (data != nullptr) *data = {};
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        // TODO: 丢弃加速度计读取时的首个无效字节，解析 11 位有符号温度并换算为摄氏度。
        if (temp != nullptr) *temp = 0.0f;
    }

}

