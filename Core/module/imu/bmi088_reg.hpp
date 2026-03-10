#pragma once

#include <cstdint>

namespace bmi088 {

// ========================= 加速度计寄存器 =========================

inline constexpr uint8_t kAccChipId           = 0x00;
inline constexpr uint8_t kAccChipIdValue      = 0x1E;

inline constexpr uint8_t kAccErrReg           = 0x02;
inline constexpr uint8_t kAccStatus           = 0x03;

inline constexpr uint8_t kAccelXoutL          = 0x12;
inline constexpr uint8_t kAccelXoutM          = 0x13;
inline constexpr uint8_t kAccelYoutL          = 0x14;
inline constexpr uint8_t kAccelYoutM          = 0x15;
inline constexpr uint8_t kAccelZoutL          = 0x16;
inline constexpr uint8_t kAccelZoutM          = 0x17;

inline constexpr uint8_t kTempM               = 0x22;
inline constexpr uint8_t kTempL               = 0x23;

inline constexpr uint8_t kAccConf             = 0x40;
inline constexpr uint8_t kAccConfMustSet      = 0x80;
inline constexpr uint8_t kAccNormal           = 0x20;  // OSR2=0x2 << 4
inline constexpr uint8_t kAcc800Hz            = 0x0B;  // ODR 800Hz
inline constexpr uint8_t kAcc1600Hz           = 0x0C;

inline constexpr uint8_t kAccRange            = 0x41;
inline constexpr uint8_t kAccRange3g          = 0x00;
inline constexpr uint8_t kAccRange6g          = 0x01;
inline constexpr uint8_t kAccRange12g         = 0x02;
inline constexpr uint8_t kAccRange24g         = 0x03;

inline constexpr uint8_t kInt1IoCtrl          = 0x53;
inline constexpr uint8_t kAccInt1IoEnable     = 0x08;  // bit3
inline constexpr uint8_t kAccInt1GpioPP       = 0x00;  // bit2=0
inline constexpr uint8_t kAccInt1GpioLow      = 0x00;  // bit1=0

inline constexpr uint8_t kIntMapData          = 0x58;
inline constexpr uint8_t kAccInt1DrdyInt      = 0x04;  // bit2

inline constexpr uint8_t kAccPwrConf          = 0x7C;
inline constexpr uint8_t kAccPwrActiveMode    = 0x00;
inline constexpr uint8_t kAccPwrSuspendMode   = 0x03;

inline constexpr uint8_t kAccPwrCtrl          = 0x7D;
inline constexpr uint8_t kAccEnableOn         = 0x04;
inline constexpr uint8_t kAccEnableOff        = 0x00;

inline constexpr uint8_t kAccSoftreset        = 0x7E;
inline constexpr uint8_t kAccSoftresetValue   = 0xB6;

// ========================= 陀螺仪寄存器 =========================

inline constexpr uint8_t kGyroChipId          = 0x00;
inline constexpr uint8_t kGyroChipIdValue     = 0x0F;

inline constexpr uint8_t kGyroXL              = 0x02;
inline constexpr uint8_t kGyroXH              = 0x03;
inline constexpr uint8_t kGyroYL              = 0x04;
inline constexpr uint8_t kGyroYH              = 0x05;
inline constexpr uint8_t kGyroZL              = 0x06;
inline constexpr uint8_t kGyroZH              = 0x07;

inline constexpr uint8_t kGyroRange           = 0x0F;
inline constexpr uint8_t kGyro2000dps         = 0x00;
inline constexpr uint8_t kGyro1000dps         = 0x01;
inline constexpr uint8_t kGyro500dps          = 0x02;
inline constexpr uint8_t kGyro250dps          = 0x03;
inline constexpr uint8_t kGyro125dps          = 0x04;

inline constexpr uint8_t kGyroBandwidth       = 0x10;
inline constexpr uint8_t kGyroBwMustSet       = 0x80;
inline constexpr uint8_t kGyro2000_532Hz      = 0x00;
inline constexpr uint8_t kGyro2000_230Hz      = 0x01;
inline constexpr uint8_t kGyro1000_116Hz      = 0x02;
inline constexpr uint8_t kGyro400_47Hz        = 0x03;
inline constexpr uint8_t kGyro200_23Hz        = 0x04;
inline constexpr uint8_t kGyro100_12Hz        = 0x05;

inline constexpr uint8_t kGyroLpm1            = 0x11;
inline constexpr uint8_t kGyroNormalMode      = 0x00;
inline constexpr uint8_t kGyroSuspendMode     = 0x80;

inline constexpr uint8_t kGyroSoftreset       = 0x14;
inline constexpr uint8_t kGyroSoftresetValue  = 0xB6;

inline constexpr uint8_t kGyroCtrl            = 0x15;
inline constexpr uint8_t kDrdyOn              = 0x80;
inline constexpr uint8_t kDrdyOff             = 0x00;

inline constexpr uint8_t kGyroInt3Int4IoConf  = 0x16;
inline constexpr uint8_t kGyroInt3GpioPP      = 0x00;  // bit1=0
inline constexpr uint8_t kGyroInt3GpioLow     = 0x00;  // bit0=0

inline constexpr uint8_t kGyroInt3Int4IoMap   = 0x18;
inline constexpr uint8_t kGyroDrdyIoInt3      = 0x01;

// ========================= 灵敏度常量 =========================

inline constexpr float kAccel3gSen   = 0.0008974358974f;
inline constexpr float kAccel6gSen   = 0.00179443359375f;
inline constexpr float kAccel12gSen  = 0.0035888671875f;
inline constexpr float kAccel24gSen  = 0.007177734375f;

inline constexpr float kGyro2000Sen  = 0.00106526443603169529841533860381f;
inline constexpr float kGyro1000Sen  = 0.00053263221801584764920766930190693f;
inline constexpr float kGyro500Sen   = 0.00026631610900792382460383465095346f;
inline constexpr float kGyro250Sen   = 0.00013315805450396191230191732547673f;
inline constexpr float kGyro125Sen   = 0.000066579027251980956150958662738366f;

// ========================= 温度解码常量 =========================

inline constexpr float kTempFactor   = 0.125f;
inline constexpr float kTempOffset   = 23.0f;

// ========================= 校准参数 =========================

inline constexpr uint16_t kCaliSampleCount   = 6000;
inline constexpr float    kCaliSampleInterval = 0.0005f;   // 0.5ms
inline constexpr float    kCaliTimeoutS       = 12.01f;
inline constexpr float    kGNormDiffThreshold = 0.5f;
inline constexpr float    kGyroDiffThreshold  = 0.15f;
inline constexpr float    kGyroOffsetThreshold = 0.01f;
inline constexpr float    kGNormExpected      = 9.8f;
inline constexpr float    kGNormReference     = 9.805f;

// ========================= 延时参数 (秒) =========================

inline constexpr float kAccelResetDelayS     = 0.150f;   // 150ms after accel soft reset
inline constexpr float kGyroResetDelayS      = 0.080f;   // 80ms after gyro soft reset
inline constexpr float kRegWriteDelayS       = 0.010f;   // 10ms between accel reg writes
inline constexpr float kRegReadDelayS        = 0.001f;   // 1ms short delay

// ========================= 错误码 =========================

enum class Bmi088Error : uint8_t {
    kNoError              = 0x00,
    kAccPwrCtrlError      = 0x01,
    kAccPwrConfError      = 0x02,
    kAccConfError         = 0x03,
    kAccSelfTestError     = 0x04,
    kAccRangeError        = 0x05,
    kInt1IoCtrlError      = 0x06,
    kIntMapDataError      = 0x07,
    kGyroRangeError       = 0x08,
    kGyroBandwidthError   = 0x09,
    kGyroLpm1Error        = 0x0A,
    kGyroCtrlError        = 0x0B,
    kGyroInt3Int4IoConfError = 0x0C,
    kGyroInt3Int4IoMapError  = 0x0D,
    kSelfTestAccelError   = 0x80,
    kSelfTestGyroError    = 0x40,
    kNoSensor             = 0xFF,
};

} // namespace bmi088
