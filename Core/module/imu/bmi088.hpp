#pragma once

#include "bmi088_reg.hpp"
#include "pid_controller.hpp"
#include "sal_spi.h"
#include "sal_gpio.h"
#include "sal_pwm.h"

#include <cstdint>

// ========================= 工作模式 =========================

enum class Bmi088WorkMode : uint8_t {
    kBlockPeriodic,   // 阻塞轮询
    kBlockTrigger,    // EXTI 触发 (ISR 内阻塞 SPI 读)
};

enum class Bmi088CaliMode : uint8_t {
    kOnline,          // 上电在线校准 (~6s)
    kPreCalibrated,   // 使用预设参数
};

// ========================= 数据结构 =========================

#pragma pack(push, 1)
struct Bmi088Data {
    float gyro[3]     = {};   // rad/s, 已减零偏
    float acc[3]      = {};   // m/s², 已校准
    float temperature = 0;    // °C
};
#pragma pack(pop)

struct Bmi088PreCali {
    float gyro_offset[3] = {0, 0, 0};
    float g_norm          = 9.805f;
};

// ========================= 配置 =========================

struct Bmi088Config {
    Bmi088WorkMode work_mode  = Bmi088WorkMode::kBlockPeriodic;
    Bmi088CaliMode cali_mode  = Bmi088CaliMode::kOnline;

    // 共享 SPI 总线
    SPI_HandleTypeDef *spi_handle = nullptr;

    // Accel CS
    GPIO_TypeDef *acc_cs_port  = nullptr;
    uint16_t      acc_cs_pin   = 0;

    // Gyro CS
    GPIO_TypeDef *gyro_cs_port = nullptr;
    uint16_t      gyro_cs_pin  = 0;

    // EXTI (仅 kBlockTrigger 模式)
    GPIO_TypeDef *acc_int_port  = nullptr;
    uint16_t      acc_int_pin   = 0;
    GPIO_TypeDef *gyro_int_port = nullptr;
    uint16_t      gyro_int_pin  = 0;

    // 加热器
    PidConfig heat_pid_config         = {};
    TIM_HandleTypeDef *heat_tim_handle = nullptr;
    uint32_t heat_tim_channel          = 0;
    float heat_target_temp             = 40.0f;

    // 预校准回退值
    Bmi088PreCali pre_cali = {};
};

// ========================= Bmi088 类 =========================

class Bmi088 {
public:
    explicit Bmi088(const Bmi088Config& cfg);

    /// 读取传感器数据, 返回 true 表示有新数据
    bool Acquire(Bmi088Data& out);

    /// 温控加热器, dt 为调用间隔 (秒)
    void HeaterCtrl(float dt);

    /// 重新校准 (需传感器静止)
    void Calibrate();

    const Bmi088Data& Data() const { return data_; }
    float GNorm() const { return g_norm_; }
    const float* GyroOffset() const { return gyro_offset_; }
    bool IsReady() const { return ready_; }

private:
    // ---- 寄存器初始化表 ----
    struct RegEntry {
        uint8_t reg;
        uint8_t value;
        uint8_t error_code;
    };

    static constexpr RegEntry kAccelInitTable[] = {
        {bmi088::kAccPwrCtrl,  bmi088::kAccEnableOn,
         static_cast<uint8_t>(bmi088::Bmi088Error::kAccPwrCtrlError)},
        {bmi088::kAccPwrConf,  bmi088::kAccPwrActiveMode,
         static_cast<uint8_t>(bmi088::Bmi088Error::kAccPwrConfError)},
        {bmi088::kAccConf,     static_cast<uint8_t>(bmi088::kAccNormal | bmi088::kAcc800Hz | bmi088::kAccConfMustSet),
         static_cast<uint8_t>(bmi088::Bmi088Error::kAccConfError)},
        {bmi088::kAccRange,    bmi088::kAccRange6g,
         static_cast<uint8_t>(bmi088::Bmi088Error::kAccRangeError)},
        {bmi088::kInt1IoCtrl,  static_cast<uint8_t>(bmi088::kAccInt1IoEnable | bmi088::kAccInt1GpioPP | bmi088::kAccInt1GpioLow),
         static_cast<uint8_t>(bmi088::Bmi088Error::kInt1IoCtrlError)},
        {bmi088::kIntMapData,  bmi088::kAccInt1DrdyInt,
         static_cast<uint8_t>(bmi088::Bmi088Error::kIntMapDataError)},
    };

    static constexpr RegEntry kGyroInitTable[] = {
        {bmi088::kGyroRange,         bmi088::kGyro2000dps,
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroRangeError)},
        {bmi088::kGyroBandwidth,     static_cast<uint8_t>(bmi088::kGyro1000_116Hz | bmi088::kGyroBwMustSet),
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroBandwidthError)},
        {bmi088::kGyroLpm1,          bmi088::kGyroNormalMode,
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroLpm1Error)},
        {bmi088::kGyroCtrl,          bmi088::kDrdyOn,
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroCtrlError)},
        {bmi088::kGyroInt3Int4IoConf, static_cast<uint8_t>(bmi088::kGyroInt3GpioPP | bmi088::kGyroInt3GpioLow),
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroInt3Int4IoConfError)},
        {bmi088::kGyroInt3Int4IoMap, bmi088::kGyroDrdyIoInt3,
         static_cast<uint8_t>(bmi088::Bmi088Error::kGyroInt3Int4IoMapError)},
    };

    // ---- SPI 读写 helpers ----
    void AccelRead(uint8_t reg, uint8_t* buf, uint8_t len);
    void GyroRead(uint8_t reg, uint8_t* buf, uint8_t len);
    void AccelWriteReg(uint8_t reg, uint8_t data);
    void GyroWriteReg(uint8_t reg, uint8_t data);

    // ---- 初始化子函数 ----
    uint8_t InitAccel();
    uint8_t InitGyro();

    // ---- 解码 ----
    void DecodeAccel(const uint8_t* buf);
    void DecodeGyro(const uint8_t* buf);
    void DecodeTemp(const uint8_t* buf);

    // ---- EXTI 回调 ----
    void OnAccelDataReady();
    void OnGyroDataReady();

    // ---- SAL 实例 ----
    sal::SPIInstance*  spi_acc_  = nullptr;
    sal::SPIInstance*  spi_gyro_ = nullptr;
    sal::GPIOInstance* acc_int_  = nullptr;
    sal::GPIOInstance* gyro_int_ = nullptr;
    sal::PWMInstance*  heat_pwm_ = nullptr;

    // ---- 温控 ----
    PidController heat_pid_;
    float heat_target_temp_ = 40.0f;

    // ---- 数据 ----
    Bmi088Data data_{};
    float gyro_offset_[3] = {};
    float g_norm_          = 0;
    float acc_coef_        = 0;
    float gyro_sen_        = 0;

    // ---- 工作模式 ----
    Bmi088WorkMode work_mode_;
    Bmi088CaliMode cali_mode_;
    Bmi088PreCali  pre_cali_;

    // ---- 触发模式标志 ----
    struct {
        uint8_t acc  : 1;
        uint8_t gyro : 1;
    } update_flag_ = {};

    bool ready_ = false;
};
