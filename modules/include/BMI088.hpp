#ifndef BMI088_HPP
#define BMI088_HPP

#include "IMU.hpp"
#include "spi.h" //< 板级生成：spi.h 里已 include main.h，故 hspi1 与 CS_Accel/CS_Gyro 引脚宏一次拿全
                 //< （README 允许 IMU 模块直接使用 HAL SPI/GPIO，这是 modules 层引 HAL 头的例外）

namespace BMI088
{
/**
 * @enum BMI088_SENSOR
 * @brief BMI088传感器的片选
 * @param BMI088_CS_ACC 加速度计片选
 * @param BMI088_CS_GYRO 陀螺仪片选
 */
enum BMI088_SENSOR
{
    BMI088_CS_ACC = 0,
    BMI088_CS_GYRO = 1,
};

/*---------------------------硬件连接---------------------------*/
// BMI088 挂在 SPI1 上：加速度计与陀螺仪共用 SCK/MISO/MOSI（PB3/PB4/PA7），靠各自独立的片选区分。
// SPI 的时序参数（Mode3、8bit、MSB first、预分频 /16）在 board/Core/Src/spi.c 的 MX_SPI1_Init 里
// 已经配好，这里只需要知道"用哪个句柄"和"拉低哪个片选"。
#define BMI088_SPI_HANDLE hspi1 //< BMI088 使用的 SPI 句柄

#define BMI088_ACC_CS_GPIO_PORT CS_Accel_GPIO_Port //< 加速度计片选端口（PA4）
#define BMI088_ACC_CS_PIN CS_Accel_Pin             //< 加速度计片选引脚
#define BMI088_GYRO_CS_GPIO_PORT CS_Gyro_GPIO_Port //< 陀螺仪片选端口（PB0）
#define BMI088_GYRO_CS_PIN CS_Gyro_Pin             //< 陀螺仪片选引脚

#define HEATING_RESISTANCE_TIM htim10            //< 加热电阻定时器
#define HEATING_RESISTANCE_CHANNEL TIM_CHANNEL_1 //< 加热电阻通道

/*-----bmi088的spi读取协议部分-----*/
#define BMI088_SPI_WRITE_CODE 0x7F
#define BMI088_SPI_READ_CODE 0x80

/*-----加速度计寄存器表-----*/
#define ACC_CHIP_ID_ADDR 0x00
#define ACC_CHIP_ID_VAL 0x1E

#define ACC_ERR_REG_ADDR 0x02

#define ACC_STATUS_ADDR 0x03

#define ACC_X_LSB_ADDR 0x12
#define ACC_X_MSB_ADDR 0x13
#define ACC_Y_LSB_ADDR 0x14
#define ACC_Y_MSB_ADDR 0x15
#define ACC_Z_LSB_ADDR 0x16
#define ACC_Z_MSB_ADDR 0x17
#define ACC_XYZ_LEN 6

#define SENSORTIME_0_ADDR 0x18
#define SENSORTIME_0_UNIT (39.0625f / 1000000.0f)
#define SENSORTIME_1_ADDR 0x19
#define SENSORTIME_1_UNIT (10.0 / 1000.0f)
#define SENSORTIME_2_ADDR 0x1A
#define SENSORTIME_2_UNIT (2.56f)
#define SENSORTIME_LEN 3

#define ACC_INT_STAT_1_ADDR 0x1D

#define TEMP_MSB_ADDR 0x22
#define TEMP_LSB_ADDR 0x23
#define TEMP_LEN 2
#define TEMP_UNIT 0.125f
#define TEMP_BIAS 23.0f

#define ACC_CONF_ADDR 0x40
#define ACC_CONF_RESERVED 0x01
/* acc_bwp 占寄存器的 [7:4]，手册 §5.3.8 给出的有效值是 0x08 / 0x09 / 0x0A
 * （注意都带 bit3）。用法是 (ACC_CONF_BWP_xxx << 4) | ACC_CONF_ODR_yyy，
 * 例：正常模式 + 800 Hz = (0x0A << 4) | 0x0B = 0xAB —— Config() 里写的就是这个值。
 * 2026-10-11 修正：这三个宏原为 0x00/0x01/0x02（少了 bit3），与手册不符；
 * 它们此前未被任何代码使用，修正后行为不变。*/
#define ACC_CONF_BWP_OSR4 0x08
#define ACC_CONF_BWP_OSR2 0x09
#define ACC_CONF_BWP_NORM 0x0A
#define ACC_CONF_ODR_12_5_Hz 0x05
#define ACC_CONF_ODR_25_Hz 0x06
#define ACC_CONF_ODR_50_Hz 0x07
#define ACC_CONF_ODR_100_Hz 0x08
#define ACC_CONF_ODR_200_Hz 0x09
#define ACC_CONF_ODR_400_Hz 0x0A
#define ACC_CONF_ODR_800_Hz 0x0B
#define ACC_CONF_ODR_1600_Hz 0x0C

#define ACC_RANGE_ADDR 0x41
#define ACC_RANGE_3G 0x00
#define ACC_RANGE_6G 0x01
#define ACC_RANGE_12G 0x02
#define ACC_RANGE_24G 0x03

#define INT1_IO_CTRL_ADDR 0x53

#define INT2_IO_CTRL_ADDR 0x54

#define INT_MAP_DATA_ADDR 0x58

#define ACC_SELF_TEST_ADDR 0x6D
#define ACC_SELF_TEST_OFF 0x00
#define ACC_SELF_TEST_POS 0x0D
#define ACC_SELF_TEST_NEG 0x09

#define ACC_PWR_CONF_ADDR 0x7C
#define ACC_PWR_CONF_SUS 0x03
#define ACC_PWR_CONF_ACT 0x00

#define ACC_PWR_CTRL_ADDR 0x7D
#define ACC_PWR_CTRL_ON 0x04
#define ACC_PWR_CTRL_OFF 0x00

#define ACC_SOFTRESET_ADDR 0x7E
#define ACC_SOFTRESET_VAL 0xB6

/*-----陀螺仪寄存器表-----*/
#define GYRO_CHIP_ID_ADDR 0x00
#define GYRO_CHIP_ID_VAL 0x0F

#define GYRO_RATE_X_LSB_ADDR 0x02
#define GYRO_RATE_X_MSB_ADDR 0x03
#define GYRO_RATE_Y_LSB_ADDR 0x04
#define GYRO_RATE_Y_MSB_ADDR 0x05
#define GYRO_RATE_Z_LSB_ADDR 0x06
#define GYRO_RATE_Z_MSB_ADDR 0x07
#define GYRO_XYZ_LEN 6

#define GYRO_INT_STAT_1_ADDR 0x0A

#define GYRO_RANGE_ADDR 0x0F
#define GYRO_RANGE_2000_DEG_S 0x00
#define GYRO_RANGE_1000_DEG_S 0x01
#define GYRO_RANGE_500_DEG_S 0x02
#define GYRO_RANGE_250_DEG_S 0x03
#define GYRO_RANGE_125_DEG_S 0x04

#define GYRO_BANDWIDTH_ADDR 0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

#define GYRO_LPM1_ADDR 0x11
#define GYRO_LPM1_NOR 0x00
#define GYRO_LPM1_SUS 0x80
#define GYRO_LPM1_DEEP_SUS 0x20

#define GYRO_SOFTRESET_ADDR 0x14
#define GYRO_SOFTRESET_VAL 0xB6

#define GYRO_INT_CTRL_ADDR 0x15

#define GYRO_INT3_INT4_IO_CONF_ADDR 0x16

#define GYRO_INT3_INT4_IO_MAP_ADDR 0x18

#define GYRO_SELF_TEST_ADDR 0x3C
#define GYRO_SELF_TEST_ON 0x01

#define GYRO_DRDY_OFF 0x00
#define GYRO_DRDY_ON 0x80

#define BMI088_GYRO_PRE_CALI_OFFSET_X -0.005280993487f
#define BMI088_GYRO_PRE_CALI_OFFSET_Y -0.000237223741f
#define BMI088_GYRO_PRE_CALI_OFFSET_Z -0.000647540528f

/* pre calibrate parameter to go here */
#define BMI088_ACCEL_PRE_CALI_OFFSET_X 0.0038458286072392397
#define BMI088_ACCEL_PRE_CALI_OFFSET_Y 0.00647039594993548f
#define BMI088_ACCEL_PRE_CALI_OFFSET_Z 0.014968990490337293f
#define BMI088_ACCEL_PRE_CALI_G_NORM 9.805f

#define IMU_ACCEL_3G_SEN 0.0008974358974f
#define IMU_ACCEL_6G_SEN 0.00179443359375f
#define IMU_ACCEL_12G_SEN 0.0035888671875f
#define IMU_ACCEL_24G_SEN 0.007177734375f

#define IMU_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define IMU_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define IMU_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define IMU_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define IMU_GYRO_125_SEN 0.000066579027251980956150958662738366f

    class cBMI088: public cIMU
    {
    public:
        /**
         * @brief 从寄存器读取数据
         * @param cs 片选
         * @param addr 寄存器地址
         * @param data 数据
         * @param len 数据长度
         */
        void ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len);

        /**
         * @brief 写数据到寄存器
         * @param cs 片选
         * @param addr 寄存器地址
         * @param data 数据
         * @param len 数据长度
         */
        void WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len);

        void Config() override;
        void Calibrate() override;

        void ReadAccData(acc_data_t *data) override;
        void ReadAccTemperature(float *temp) override;
        void ReadGyroData(gyro_data_t *data) override;

        void VerifyAccChipID() override;
        void VerifyGyroChipID() override;
        void VerifyAccData() override;
        void VerifyGyroData() override;

        void TemperatureControl(float target_temp) override;



    private:
        /* 陀螺仪零偏（rad/s）。初值给预标定常数，而不是留空 ——
         * 否则 Calibrate() 没跑（例如 ID 校验失败提前返回）时，
         * ReadGyroData() 会减到未初始化内存，直接算出 NaN 污染上层。*/
        float Gyro_offset[3] = {BMI088_GYRO_PRE_CALI_OFFSET_X,
                                BMI088_GYRO_PRE_CALI_OFFSET_Y,
                                BMI088_GYRO_PRE_CALI_OFFSET_Z};
    };

}

#endif // BMI088_HPP
