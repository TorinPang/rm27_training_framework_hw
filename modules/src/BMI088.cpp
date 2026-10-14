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

        /* 判断一段原始字节是否“通信失效”：MISO 断线/被短接到电源或地、总线被拉死时，
         * SPI 读回的是恒定字（全 0x00 或全 0xFF），于是所有字节完全相同。
         * 真实数据不会这样 —— 三轴各自独立，且各轴最低位一直在噪声里跳动，
         * 连续 6 个字节（含 3 个 LSB）全同的概率极低。
         * 用这个判据与板子姿态无关；若改用“|a| ≈ 9.8”判，做直线加速运动时会误报。*/
        bool RawIsDead(const uint8_t *data, uint8_t n)
        {
            for (uint8_t i = 1U; i < n; ++i)
            {
                if (data[i] != data[0]) return false;
            }
            return true;
        }
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
        /* ---- 量纲口径（本轮定下，写在这里备查）----
         * cIMU 里的 TempPid 是老师给的：PID(0.1, 0, 0, maxOut=25000, maxIOut=3)。
         * maxOut=25000 与 PWM 要的 duty∈[0,1] 明显不是同一个量纲，PID 自身的输出限幅
         * 在本用途下等于不起作用。这里选择**不动老师的参数**，而是在本函数内把 PID 输出
         * 按"1.0 = 满功率"显式归一化到 [0,1] 再下发：
         *     duty = clamp(TempPid.result, 0, 1)
         * 于是 kp=0.1 的含义就是"每 1 ℃ 误差给 10 % 功率"（误差 10 ℃ 即满功率），
         * 这对加热电阻是合理的 P 增益，也说明 25000 只是个够大的兜底值。
         * 另外两条理由说明这样是安全的：
         *   1) 加热只升温，不制冷 —— 负误差（已超温）一律截到 0；
         *   2) 本轮 ki=kd=0（纯 P），maxIOut 不参与运算；将来若整定出积分项，
         *      末端这道夹紧仍保证下发值一定合法。
         * 代价：若以后带积分项，PID 的 maxIOut 与这里的 [0,1] 量纲仍需再对齐一次。*/

        /* ---- 不安全的工况一律不加热，并把 PID 清干净 ---- */
        const bool temp_valid = std::isfinite(acc_data.temperature) &&
                                (acc_data.temperature > TEMP_SANE_MIN_C) &&
                                (acc_data.temperature < TEMP_SANE_MAX_C);

        if (self_test.INIT_ERR || self_test.ACC_DATA_ERR || !temp_valid ||
            !std::isfinite(target_temp))
        {
            TempPid.Clear(); // 清掉累加量，避免带旧的积分参与下一次运算
            self_test.TEMP_CTRL_ERR = true;
            PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
            return;
        }

        TempPid.ref = target_temp;
        TempPid.fdb = acc_data.temperature;
        TempPid.UpdateResult();

        /* 显式归一化到 [0,1]：负（含 NaN）→ 0，> 1 → 1 */
        float duty = TempPid.result;
        if (!(duty > 0.0f)) duty = 0.0f; // 这样写同时挡住 NaN
        if (duty > 1.0f) duty = 1.0f;

        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
        self_test.TEMP_CTRL_ERR = false;
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
        /* 重新取一次原始字节来判通信是否失效（与 ReadAccData 同一读协议：
         * 加速度计地址后还有 1 个哑字节，故读 1+6 = 7 字节，rx+1 才是数据）。
         * 独立读一次的好处是 Verify 不依赖 Read 是否刚跑过，可单独调用做通信自检。*/
        uint8_t rx[ACC_XYZ_LEN + 1U];
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, rx, ACC_XYZ_LEN + 1U);

        const bool comm_dead = RawIsDead(rx + 1U, ACC_XYZ_LEN);
        const bool not_finite = !(std::isfinite(acc_data.x) &&
                                  std::isfinite(acc_data.y) &&
                                  std::isfinite(acc_data.z));

        self_test.ACC_DATA_ERR = comm_dead || not_finite;
    }

    void cBMI088::VerifyGyroData()
    {
        /* 陀螺仪没有哑字节，数据从 rx[0] 起。*/
        uint8_t rx[GYRO_XYZ_LEN];
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, rx, GYRO_XYZ_LEN);

        const bool comm_dead = RawIsDead(rx, GYRO_XYZ_LEN);
        const bool not_finite = !(std::isfinite(gyro_data.x) &&
                                  std::isfinite(gyro_data.y) &&
                                  std::isfinite(gyro_data.z));

        self_test.GYRO_DATA_ERR = comm_dead || not_finite;
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
        if (data == nullptr) return;

        /* 三轴 burst 读：加速度计在地址之后先送 1 个哑字节，随后依次是
         * X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB —— 共 1+6 = 7 字节。
         * 每轴“低字节在前”，按 (MSB<<8)|LSB 拼成有符号 16 位原始计数。*/
        uint8_t rx[ACC_XYZ_LEN + 1U];
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, rx, ACC_XYZ_LEN + 1U);

        const int16_t raw_x = static_cast<int16_t>((static_cast<uint16_t>(rx[2]) << 8) | rx[1]);
        const int16_t raw_y = static_cast<int16_t>((static_cast<uint16_t>(rx[4]) << 8) | rx[3]);
        const int16_t raw_z = static_cast<int16_t>((static_cast<uint16_t>(rx[6]) << 8) | rx[5]);

        /* 量纲：raw × IMU_ACCEL_6G_SEN = m/s²。
         * 该常数就是“每 LSB 多少 m/s²”（6 g 满量程 / 32768 LSB，已把 g 折算进去），
         * 所以结果直接是 m/s²，不需要再乘 9.8。随后扣掉预标定零偏（同为 m/s² 量纲）。*/
        data->x = static_cast<float>(raw_x) * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_X;
        data->y = static_cast<float>(raw_y) * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Y;
        data->z = static_cast<float>(raw_z) * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Z;
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        if (data == nullptr) return;

        /* 陀螺仪 burst 读**没有**哑字节（数据手册 §6.1.1），地址之后当场就是
         * X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB，共 6 字节。*/
        uint8_t rx[GYRO_XYZ_LEN];
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, rx, GYRO_XYZ_LEN);

        const int16_t raw_x = static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[0]);
        const int16_t raw_y = static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[2]);
        const int16_t raw_z = static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[4]);

        /* 量纲：raw × IMU_GYRO_2000_SEN = rad/s。
         * 该常数 = (2000 °/s ÷ 32768) × π/180，已含 deg→rad 换算，不用再乘 π/180。
         * 再减掉 Calibrate() 标出的零偏（同为 rad/s）。若这次上电没标定过，
         * Gyro_offset 的初值是头文件里的预标定常数，不会是野值。*/
        data->x = static_cast<float>(raw_x) * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = static_cast<float>(raw_y) * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = static_cast<float>(raw_z) * IMU_GYRO_2000_SEN - Gyro_offset[2];
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        if (temp == nullptr) return;

        /* 温度寄存器 0x22(MSB)/0x23(LSB) 走的仍是加速度计接口，所以同样有 1 个哑字节：
         * 读 1+2 = 3 字节。有效值是 11 位有符号、左对齐 —— 把 MSB 与 LSB 的高 3 位
         * 拼成 16 位后算术右移 5 位，得到 11 位计数（可为负）。
         * 换算：count × 0.125 ℃ + 23 ℃。*/
        uint8_t rx[TEMP_LEN + 1U];
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, rx, TEMP_LEN + 1U);

        const int16_t raw = static_cast<int16_t>(
            (static_cast<uint16_t>(rx[1]) << 8) | (rx[2] & 0xE0U));

        *temp = static_cast<float>(raw >> 5) * TEMP_UNIT + TEMP_BIAS;
    }

}

