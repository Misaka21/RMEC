#pragma once

#include "bmi088_reg.hpp"
#include "pid_controller.hpp"
#include "sal_spi.h"
#include "sal_gpio.h"
#include "sal_pwm.h"

#include <cstdint>

// ========================= 工作模式 =========================

enum class Bmi088WorkMode : uint8_t {
    BLOCK_PERIODIC,   // 阻塞轮询
    BLOCK_TRIGGER,    // EXTI 触发 (ISR 内阻塞 SPI 读)
};

enum class Bmi088CaliMode : uint8_t {
    ONLINE,           // 上电在线校准 (~6s)
    PRE_CALIBRATED,   // 使用预设参数
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
    Bmi088WorkMode work_mode  = Bmi088WorkMode::BLOCK_PERIODIC;
    Bmi088CaliMode cali_mode  = Bmi088CaliMode::ONLINE;

    // 共享 SPI 总线
    SPI_HandleTypeDef *spi_handle = nullptr;

    // Accel CS
    GPIO_TypeDef *acc_cs_port  = nullptr;
    uint16_t      acc_cs_pin   = 0;

    // Gyro CS
    GPIO_TypeDef *gyro_cs_port = nullptr;
    uint16_t      gyro_cs_pin  = 0;

    // EXTI (仅 BLOCK_TRIGGER 模式)
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

    static constexpr RegEntry ACCEL_INIT_TABLE[] = {
        {bmi088::ACC_PWR_CTRL,  bmi088::ACC_ENABLE_ON,
         static_cast<uint8_t>(bmi088::Bmi088Error::ACC_PWR_CTRL_ERROR)},
        {bmi088::ACC_PWR_CONF,  bmi088::ACC_PWR_ACTIVE_MODE,
         static_cast<uint8_t>(bmi088::Bmi088Error::ACC_PWR_CONF_ERROR)},
        {bmi088::ACC_CONF,     static_cast<uint8_t>(bmi088::ACC_NORMAL | bmi088::ACC_800HZ | bmi088::ACC_CONF_MUST_SET),
         static_cast<uint8_t>(bmi088::Bmi088Error::ACC_CONF_ERROR)},
        {bmi088::ACC_RANGE,    bmi088::ACC_RANGE_6G,
         static_cast<uint8_t>(bmi088::Bmi088Error::ACC_RANGE_ERROR)},
        {bmi088::INT1_IO_CTRL,  static_cast<uint8_t>(bmi088::ACC_INT1_IO_ENABLE | bmi088::ACC_INT1_GPIO_PP | bmi088::ACC_INT1_GPIO_LOW),
         static_cast<uint8_t>(bmi088::Bmi088Error::INT1_IO_CTRL_ERROR)},
        {bmi088::INT_MAP_DATA,  bmi088::ACC_INT1_DRDY_INT,
         static_cast<uint8_t>(bmi088::Bmi088Error::INT_MAP_DATA_ERROR)},
    };

    static constexpr RegEntry GYRO_INIT_TABLE[] = {
        {bmi088::GYRO_RANGE,         bmi088::GYRO_2000DPS,
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_RANGE_ERROR)},
        {bmi088::GYRO_BANDWIDTH,     static_cast<uint8_t>(bmi088::GYRO_1000_116HZ | bmi088::GYRO_BW_MUST_SET),
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_BANDWIDTH_ERROR)},
        {bmi088::GYRO_LPM1,          bmi088::GYRO_NORMAL_MODE,
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_LPM1_ERROR)},
        {bmi088::GYRO_CTRL,          bmi088::DRDY_ON,
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_CTRL_ERROR)},
        {bmi088::GYRO_INT3_INT4_IO_CONF, static_cast<uint8_t>(bmi088::GYRO_INT3_GPIO_PP | bmi088::GYRO_INT3_GPIO_LOW),
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_INT3_INT4_IO_CONF_ERROR)},
        {bmi088::GYRO_INT3_INT4_IO_MAP, bmi088::GYRO_DRDY_IO_INT3,
         static_cast<uint8_t>(bmi088::Bmi088Error::GYRO_INT3_INT4_IO_MAP_ERROR)},
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
    sal::SpiInstance*  spi_acc_  = nullptr;
    sal::SpiInstance*  spi_gyro_ = nullptr;
    sal::GpioInstance* acc_int_  = nullptr;
    sal::GpioInstance* gyro_int_ = nullptr;
    sal::PwmInstance*  heat_pwm_ = nullptr;

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
