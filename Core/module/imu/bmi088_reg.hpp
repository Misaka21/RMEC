#pragma once

#include <cstdint>

namespace bmi088 {

// ========================= 加速度计寄存器 =========================

inline constexpr uint8_t ACC_CHIP_ID           = 0x00;
inline constexpr uint8_t ACC_CHIP_ID_VALUE      = 0x1E;

inline constexpr uint8_t ACC_ERR_REG           = 0x02;
inline constexpr uint8_t ACC_STATUS           = 0x03;

inline constexpr uint8_t ACCEL_XOUT_L          = 0x12;
inline constexpr uint8_t ACCEL_XOUT_M          = 0x13;
inline constexpr uint8_t ACCEL_YOUT_L          = 0x14;
inline constexpr uint8_t ACCEL_YOUT_M          = 0x15;
inline constexpr uint8_t ACCEL_ZOUT_L          = 0x16;
inline constexpr uint8_t ACCEL_ZOUT_M          = 0x17;

inline constexpr uint8_t TEMP_M               = 0x22;
inline constexpr uint8_t TEMP_L               = 0x23;

inline constexpr uint8_t ACC_CONF             = 0x40;
inline constexpr uint8_t ACC_CONF_MUST_SET      = 0x80;
inline constexpr uint8_t ACC_NORMAL           = 0x20;  // OSR2=0x2 << 4
inline constexpr uint8_t ACC_800HZ            = 0x0B;  // ODR 800Hz
inline constexpr uint8_t ACC_1600HZ           = 0x0C;

inline constexpr uint8_t ACC_RANGE            = 0x41;
inline constexpr uint8_t ACC_RANGE_3G          = 0x00;
inline constexpr uint8_t ACC_RANGE_6G          = 0x01;
inline constexpr uint8_t ACC_RANGE_12G         = 0x02;
inline constexpr uint8_t ACC_RANGE_24G         = 0x03;

inline constexpr uint8_t INT1_IO_CTRL          = 0x53;
inline constexpr uint8_t ACC_INT1_IO_ENABLE     = 0x08;  // bit3
inline constexpr uint8_t ACC_INT1_GPIO_PP       = 0x00;  // bit2=0
inline constexpr uint8_t ACC_INT1_GPIO_LOW      = 0x00;  // bit1=0

inline constexpr uint8_t INT_MAP_DATA          = 0x58;
inline constexpr uint8_t ACC_INT1_DRDY_INT      = 0x04;  // bit2

inline constexpr uint8_t ACC_PWR_CONF          = 0x7C;
inline constexpr uint8_t ACC_PWR_ACTIVE_MODE    = 0x00;
inline constexpr uint8_t ACC_PWR_SUSPEND_MODE   = 0x03;

inline constexpr uint8_t ACC_PWR_CTRL          = 0x7D;
inline constexpr uint8_t ACC_ENABLE_ON         = 0x04;
inline constexpr uint8_t ACC_ENABLE_OFF        = 0x00;

inline constexpr uint8_t ACC_SOFTRESET        = 0x7E;
inline constexpr uint8_t ACC_SOFTRESET_VALUE   = 0xB6;

// ========================= 陀螺仪寄存器 =========================

inline constexpr uint8_t GYRO_CHIP_ID          = 0x00;
inline constexpr uint8_t GYRO_CHIP_ID_VALUE     = 0x0F;

inline constexpr uint8_t GYRO_XL              = 0x02;
inline constexpr uint8_t GYRO_XH              = 0x03;
inline constexpr uint8_t GYRO_YL              = 0x04;
inline constexpr uint8_t GYRO_YH              = 0x05;
inline constexpr uint8_t GYRO_ZL              = 0x06;
inline constexpr uint8_t GYRO_ZH              = 0x07;

inline constexpr uint8_t GYRO_RANGE           = 0x0F;
inline constexpr uint8_t GYRO_2000DPS         = 0x00;
inline constexpr uint8_t GYRO_1000DPS         = 0x01;
inline constexpr uint8_t GYRO_500DPS          = 0x02;
inline constexpr uint8_t GYRO_250DPS          = 0x03;
inline constexpr uint8_t GYRO_125DPS          = 0x04;

inline constexpr uint8_t GYRO_BANDWIDTH       = 0x10;
inline constexpr uint8_t GYRO_BW_MUST_SET       = 0x80;
inline constexpr uint8_t GYRO_2000_532HZ      = 0x00;
inline constexpr uint8_t GYRO_2000_230HZ      = 0x01;
inline constexpr uint8_t GYRO_1000_116HZ      = 0x02;
inline constexpr uint8_t GYRO_400_47HZ        = 0x03;
inline constexpr uint8_t GYRO_200_23HZ        = 0x04;
inline constexpr uint8_t GYRO_100_12HZ        = 0x05;

inline constexpr uint8_t GYRO_LPM1            = 0x11;
inline constexpr uint8_t GYRO_NORMAL_MODE      = 0x00;
inline constexpr uint8_t GYRO_SUSPEND_MODE     = 0x80;

inline constexpr uint8_t GYRO_SOFTRESET       = 0x14;
inline constexpr uint8_t GYRO_SOFTRESET_VALUE  = 0xB6;

inline constexpr uint8_t GYRO_CTRL            = 0x15;
inline constexpr uint8_t DRDY_ON              = 0x80;
inline constexpr uint8_t DRDY_OFF             = 0x00;

inline constexpr uint8_t GYRO_INT3_INT4_IO_CONF  = 0x16;
inline constexpr uint8_t GYRO_INT3_GPIO_PP      = 0x00;  // bit1=0
inline constexpr uint8_t GYRO_INT3_GPIO_LOW     = 0x00;  // bit0=0

inline constexpr uint8_t GYRO_INT3_INT4_IO_MAP   = 0x18;
inline constexpr uint8_t GYRO_DRDY_IO_INT3      = 0x01;

// ========================= 灵敏度常量 =========================

inline constexpr float ACCEL_3G_SEN   = 0.0008974358974f;
inline constexpr float ACCEL_6G_SEN   = 0.00179443359375f;
inline constexpr float ACCEL_12G_SEN  = 0.0035888671875f;
inline constexpr float ACCEL_24G_SEN  = 0.007177734375f;

inline constexpr float GYRO_2000_SEN  = 0.00106526443603169529841533860381f;
inline constexpr float GYRO_1000_SEN  = 0.00053263221801584764920766930190693f;
inline constexpr float GYRO_500_SEN   = 0.00026631610900792382460383465095346f;
inline constexpr float GYRO_250_SEN   = 0.00013315805450396191230191732547673f;
inline constexpr float GYRO_125_SEN   = 0.000066579027251980956150958662738366f;

// ========================= 温度解码常量 =========================

inline constexpr float TEMP_FACTOR   = 0.125f;
inline constexpr float TEMP_OFFSET   = 23.0f;

// ========================= 校准参数 =========================

inline constexpr uint16_t CALI_SAMPLE_COUNT   = 6000;
inline constexpr float    CALI_SAMPLE_INTERVAL = 0.0005f;   // 0.5ms
inline constexpr float    CALI_TIMEOUT_S       = 12.01f;
inline constexpr float    G_NORM_DIFF_THRESHOLD = 0.5f;
inline constexpr float    GYRO_DIFF_THRESHOLD  = 0.15f;
inline constexpr float    GYRO_OFFSET_THRESHOLD = 0.01f;
inline constexpr float    G_NORM_EXPECTED      = 9.8f;
inline constexpr float    G_NORM_REFERENCE     = 9.805f;

// ========================= 延时参数 (秒) =========================

inline constexpr float ACCEL_RESET_DELAY_S     = 0.150f;   // 150ms after accel soft reset
inline constexpr float GYRO_RESET_DELAY_S      = 0.080f;   // 80ms after gyro soft reset
inline constexpr float REG_WRITE_DELAY_S       = 0.010f;   // 10ms between accel reg writes
inline constexpr float REG_READ_DELAY_S        = 0.001f;   // 1ms short delay

// ========================= 错误码 =========================

enum class Bmi088Error : uint8_t {
    NO_ERROR              = 0x00,
    ACC_PWR_CTRL_ERROR      = 0x01,
    ACC_PWR_CONF_ERROR      = 0x02,
    ACC_CONF_ERROR         = 0x03,
    ACC_SELF_TEST_ERROR     = 0x04,
    ACC_RANGE_ERROR        = 0x05,
    INT1_IO_CTRL_ERROR      = 0x06,
    INT_MAP_DATA_ERROR      = 0x07,
    GYRO_RANGE_ERROR       = 0x08,
    GYRO_BANDWIDTH_ERROR   = 0x09,
    GYRO_LPM1_ERROR        = 0x0A,
    GYRO_CTRL_ERROR        = 0x0B,
    GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,
    SELF_TEST_ACCEL_ERROR   = 0x80,
    SELF_TEST_GYRO_ERROR    = 0x40,
    NO_SENSOR             = 0xFF,
};

} // namespace bmi088
