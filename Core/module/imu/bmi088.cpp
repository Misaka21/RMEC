#include "bmi088.hpp"
#include "bmi088_reg.hpp"
#include "general_def.hpp"
#include "sal_dwt.h"

#include <cmath>
#include <cstring>

// ========================= 静态初始化表定义 =========================

constexpr Bmi088::RegEntry Bmi088::kAccelInitTable[];
constexpr Bmi088::RegEntry Bmi088::kGyroInitTable[];

// ========================= SPI 读写 helpers =========================

void Bmi088::AccelRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    // BMI088 accel SPI 协议: tx[0]=0x80|reg, tx[1]=dummy, rx[2..] = data
    uint8_t tx[8] = {};
    uint8_t rx[8] = {};
    tx[0] = 0x80u | reg;
    spi_acc_->CSReset();
    spi_acc_->SPITransmitReceive(tx, rx, len + 2);
    spi_acc_->CSSet();
    for (uint8_t i = 0; i < len; ++i)
        buf[i] = rx[i + 2];
}

void Bmi088::GyroRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    // BMI088 gyro SPI 协议: tx[0]=0x80|reg, rx[1..] = data (无 dummy byte)
    uint8_t tx[8] = {};
    uint8_t rx[8] = {};
    tx[0] = 0x80u | reg;
    spi_gyro_->CSReset();
    spi_gyro_->SPITransmitReceive(tx, rx, len + 1);
    spi_gyro_->CSSet();
    for (uint8_t i = 0; i < len; ++i)
        buf[i] = rx[i + 1];
}

void Bmi088::AccelWriteReg(uint8_t reg, uint8_t data) {
    spi_acc_->WriteReg(reg, data);
}

void Bmi088::GyroWriteReg(uint8_t reg, uint8_t data) {
    spi_gyro_->WriteReg(reg, data);
}

// ========================= 初始化子函数 =========================

uint8_t Bmi088::InitAccel() {
    uint8_t whoami = 0;

    // accel 上电默认 I2C 模式, 做一次 dummy read 切换到 SPI 模式
    AccelRead(bmi088::kAccChipId, &whoami, 1);
    DWTInstance::DWTDelay(bmi088::kRegReadDelayS);

    // 软复位
    AccelWriteReg(bmi088::kAccSoftreset, bmi088::kAccSoftresetValue);
    DWTInstance::DWTDelay(bmi088::kAccelResetDelayS);

    // 验证 Chip ID
    AccelRead(bmi088::kAccChipId, &whoami, 1);
    if (whoami != bmi088::kAccChipIdValue)
        return static_cast<uint8_t>(bmi088::Bmi088Error::kNoSensor);
    DWTInstance::DWTDelay(bmi088::kRegReadDelayS);

    // 写寄存器表并回读验证
    uint8_t error = 0;
    for (const auto& entry : kAccelInitTable) {
        AccelWriteReg(entry.reg, entry.value);
        DWTInstance::DWTDelay(bmi088::kRegWriteDelayS);

        uint8_t readback = 0;
        AccelRead(entry.reg, &readback, 1);
        DWTInstance::DWTDelay(bmi088::kRegWriteDelayS);

        if (readback != entry.value)
            error |= entry.error_code;
    }
    return error;
}

uint8_t Bmi088::InitGyro() {
    // 软复位
    GyroWriteReg(bmi088::kGyroSoftreset, bmi088::kGyroSoftresetValue);
    DWTInstance::DWTDelay(bmi088::kGyroResetDelayS);

    // 验证 Chip ID
    uint8_t whoami = 0;
    GyroRead(bmi088::kGyroChipId, &whoami, 1);
    if (whoami != bmi088::kGyroChipIdValue)
        return static_cast<uint8_t>(bmi088::Bmi088Error::kNoSensor);
    DWTInstance::DWTDelay(bmi088::kRegReadDelayS);

    // 写寄存器表并回读验证
    uint8_t error = 0;
    for (const auto& entry : kGyroInitTable) {
        GyroWriteReg(entry.reg, entry.value);
        DWTInstance::DWTDelay(bmi088::kRegReadDelayS);

        uint8_t readback = 0;
        GyroRead(entry.reg, &readback, 1);
        DWTInstance::DWTDelay(bmi088::kRegReadDelayS);

        if (readback != entry.value)
            error |= entry.error_code;
    }
    return error;
}

// ========================= 数据解码 =========================

void Bmi088::DecodeAccel(const uint8_t* buf) {
    for (uint8_t i = 0; i < 3; ++i) {
        auto raw = static_cast<int16_t>(
            (static_cast<uint16_t>(buf[2 * i + 1]) << 8) | buf[2 * i]);
        data_.acc[i] = acc_coef_ * static_cast<float>(raw);
    }
}

void Bmi088::DecodeGyro(const uint8_t* buf) {
    for (uint8_t i = 0; i < 3; ++i) {
        auto raw = static_cast<int16_t>(
            (static_cast<uint16_t>(buf[2 * i + 1]) << 8) | buf[2 * i]);
        data_.gyro[i] = gyro_sen_ * static_cast<float>(raw) - gyro_offset_[i];
    }
}

void Bmi088::DecodeTemp(const uint8_t* buf) {
    auto raw = static_cast<int16_t>(
        (static_cast<uint16_t>(buf[0]) << 3) | (buf[1] >> 5));
    if (raw > 1023)
        raw -= 2048;
    data_.temperature = static_cast<float>(raw) * bmi088::kTempFactor + bmi088::kTempOffset;
}

// ========================= EXTI 回调 =========================

void Bmi088::OnAccelDataReady() {
    uint8_t buf[6] = {};
    AccelRead(bmi088::kAccelXoutL, buf, 6);
    DecodeAccel(buf);

    uint8_t tbuf[2] = {};
    AccelRead(bmi088::kTempM, tbuf, 2);
    DecodeTemp(tbuf);

    update_flag_.acc = 1;
}

void Bmi088::OnGyroDataReady() {
    uint8_t buf[6] = {};
    GyroRead(bmi088::kGyroXL, buf, 6);
    DecodeGyro(buf);
    update_flag_.gyro = 1;
}

// ========================= 构造函数 =========================

Bmi088::Bmi088(const Bmi088Config& cfg)
    : heat_target_temp_(cfg.heat_target_temp),
      work_mode_(cfg.work_mode),
      cali_mode_(cfg.cali_mode),
      pre_cali_(cfg.pre_cali)
{
    // 1. 创建 2 个 SPI 实例 (共享 SPI handle, 不同 CS)
    sal::SPIInstance::SPIConfig acc_spi_cfg{};
    acc_spi_cfg.handle    = cfg.spi_handle;
    acc_spi_cfg.xfer_type = sal::SPI_XFER_BLOCK;
    acc_spi_cfg.cs_port   = cfg.acc_cs_port;
    acc_spi_cfg.cs_pin    = cfg.acc_cs_pin;
    spi_acc_ = new sal::SPIInstance(acc_spi_cfg);

    sal::SPIInstance::SPIConfig gyro_spi_cfg{};
    gyro_spi_cfg.handle    = cfg.spi_handle;
    gyro_spi_cfg.xfer_type = sal::SPI_XFER_BLOCK;
    gyro_spi_cfg.cs_port   = cfg.gyro_cs_port;
    gyro_spi_cfg.cs_pin    = cfg.gyro_cs_pin;
    spi_gyro_ = new sal::SPIInstance(gyro_spi_cfg);

    // 2. 创建 PWM 实例 + 初始化 PID
    if (cfg.heat_tim_handle != nullptr) {
        sal::PWMInstance::PWMConfig pwm_cfg{};
        pwm_cfg.handle  = cfg.heat_tim_handle;
        pwm_cfg.channel = cfg.heat_tim_channel;
        heat_pwm_ = new sal::PWMInstance(pwm_cfg);
        heat_pwm_->Start();
    }
    heat_pid_.Init(cfg.heat_pid_config);

    // 3. 重试初始化直到成功
    uint8_t error;
    do {
        error = 0;
        error |= InitAccel();
        error |= InitGyro();
    } while (error != 0);

    // 4. 临时切换阻塞模式执行校准
    auto saved_mode = work_mode_;
    work_mode_ = Bmi088WorkMode::kBlockPeriodic;
    Calibrate();
    work_mode_ = saved_mode;

    // 5. 若为触发模式, 注册 EXTI
    if (work_mode_ == Bmi088WorkMode::kBlockTrigger) {
        sal::GPIOInstance::GPIOConfig acc_int_cfg{};
        acc_int_cfg.port     = cfg.acc_int_port;
        acc_int_cfg.pin      = cfg.acc_int_pin;
        acc_int_cfg.exti_cbk = [this]() { OnAccelDataReady(); };
        acc_int_ = new sal::GPIOInstance(acc_int_cfg);

        sal::GPIOInstance::GPIOConfig gyro_int_cfg{};
        gyro_int_cfg.port     = cfg.gyro_int_port;
        gyro_int_cfg.pin      = cfg.gyro_int_pin;
        gyro_int_cfg.exti_cbk = [this]() { OnGyroDataReady(); };
        gyro_int_ = new sal::GPIOInstance(gyro_int_cfg);
    }

    ready_ = true;
}

// ========================= Acquire =========================

bool Bmi088::Acquire(Bmi088Data& out) {
    if (work_mode_ == Bmi088WorkMode::kBlockPeriodic) {
        // 阻塞读取
        uint8_t buf[6] = {};

        AccelRead(bmi088::kAccelXoutL, buf, 6);
        DecodeAccel(buf);

        GyroRead(bmi088::kGyroXL, buf, 6);
        DecodeGyro(buf);

        uint8_t tbuf[2] = {};
        AccelRead(bmi088::kTempM, tbuf, 2);
        DecodeTemp(tbuf);

        out = data_;
        return true;
    }

    // 触发模式: 检查 ISR 缓存
    if (work_mode_ == Bmi088WorkMode::kBlockTrigger) {
        bool updated = false;
        if (update_flag_.acc) {
            out.acc[0] = data_.acc[0];
            out.acc[1] = data_.acc[1];
            out.acc[2] = data_.acc[2];
            out.temperature = data_.temperature;
            update_flag_.acc = 0;
            updated = true;
        }
        if (update_flag_.gyro) {
            out.gyro[0] = data_.gyro[0];
            out.gyro[1] = data_.gyro[1];
            out.gyro[2] = data_.gyro[2];
            update_flag_.gyro = 0;
            updated = true;
        }
        return updated;
    }

    return false;
}

// ========================= 温控加热器 =========================

void Bmi088::HeaterCtrl(float dt) {
    if (heat_pwm_ == nullptr)
        return;
    float out = heat_pid_.Calculate(data_.temperature, heat_target_temp_, dt);
    heat_pwm_->SetDutyCycle(Clamp(out, 0.0f, 1.0f));
}

// ========================= 校准 =========================

static float NormOf3d(const float v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Bmi088::Calibrate() {
    if (cali_mode_ == Bmi088CaliMode::kOnline) {
        acc_coef_  = bmi088::kAccel6gSen;
        gyro_sen_  = bmi088::kGyro2000Sen;

        float start_time = DWTInstance::DWTGetTimeline_s();
        float gyro_max[3], gyro_min[3];
        float g_norm_temp, g_norm_max, g_norm_min;
        float gyro_diff[3], g_norm_diff;

        Bmi088Data raw{};

        do {
            // 超时回退到预校准
            if (DWTInstance::DWTGetTimeline_s() - start_time > bmi088::kCaliTimeoutS) {
                cali_mode_ = Bmi088CaliMode::kPreCalibrated;
                break;
            }

            DWTInstance::DWTDelay(bmi088::kCaliSampleInterval);
            g_norm_ = 0;
            gyro_offset_[0] = gyro_offset_[1] = gyro_offset_[2] = 0;
            g_norm_diff = 0;
            gyro_diff[0] = gyro_diff[1] = gyro_diff[2] = 0;

            for (uint16_t i = 0; i < bmi088::kCaliSampleCount; ++i) {
                // 校准时临时清零 offset, 读到的是 raw 数据
                float saved_offset[3] = {gyro_offset_[0], gyro_offset_[1], gyro_offset_[2]};
                gyro_offset_[0] = gyro_offset_[1] = gyro_offset_[2] = 0;

                Acquire(raw);

                gyro_offset_[0] = saved_offset[0];
                gyro_offset_[1] = saved_offset[1];
                gyro_offset_[2] = saved_offset[2];

                g_norm_temp = NormOf3d(raw.acc);
                g_norm_ += g_norm_temp;

                // 校准时 gyro 数据未减 offset (因 offset 被清零), 直接累加
                gyro_offset_[0] += raw.gyro[0];
                gyro_offset_[1] += raw.gyro[1];
                gyro_offset_[2] += raw.gyro[2];

                if (i == 0) {
                    g_norm_max = g_norm_min = g_norm_temp;
                    for (uint8_t j = 0; j < 3; ++j)
                        gyro_max[j] = gyro_min[j] = raw.gyro[j];
                } else {
                    if (g_norm_temp > g_norm_max) g_norm_max = g_norm_temp;
                    if (g_norm_temp < g_norm_min) g_norm_min = g_norm_temp;
                    for (uint8_t j = 0; j < 3; ++j) {
                        if (raw.gyro[j] > gyro_max[j]) gyro_max[j] = raw.gyro[j];
                        if (raw.gyro[j] < gyro_min[j]) gyro_min[j] = raw.gyro[j];
                    }
                }

                g_norm_diff = g_norm_max - g_norm_min;
                for (uint8_t j = 0; j < 3; ++j)
                    gyro_diff[j] = gyro_max[j] - gyro_min[j];

                // 运动检测: 超出阈值则提前退出内循环并重试
                if (g_norm_diff > bmi088::kGNormDiffThreshold ||
                    gyro_diff[0] > bmi088::kGyroDiffThreshold ||
                    gyro_diff[1] > bmi088::kGyroDiffThreshold ||
                    gyro_diff[2] > bmi088::kGyroDiffThreshold)
                    break;

                DWTInstance::DWTDelay(bmi088::kCaliSampleInterval);
            }

            // 求均值
            g_norm_ /= static_cast<float>(bmi088::kCaliSampleCount);
            for (uint8_t i = 0; i < 3; ++i)
                gyro_offset_[i] /= static_cast<float>(bmi088::kCaliSampleCount);

        } while (g_norm_diff > bmi088::kGNormDiffThreshold ||
                 std::fabs(g_norm_ - bmi088::kGNormExpected) > bmi088::kGNormDiffThreshold ||
                 gyro_diff[0] > bmi088::kGyroDiffThreshold ||
                 gyro_diff[1] > bmi088::kGyroDiffThreshold ||
                 gyro_diff[2] > bmi088::kGyroDiffThreshold ||
                 std::fabs(gyro_offset_[0]) > bmi088::kGyroOffsetThreshold ||
                 std::fabs(gyro_offset_[1]) > bmi088::kGyroOffsetThreshold ||
                 std::fabs(gyro_offset_[2]) > bmi088::kGyroOffsetThreshold);
    }

    // 离线校准 (或在线超时回退)
    if (cali_mode_ == Bmi088CaliMode::kPreCalibrated) {
        acc_coef_  = bmi088::kAccel6gSen;
        gyro_sen_  = bmi088::kGyro2000Sen;
        gyro_offset_[0] = pre_cali_.gyro_offset[0];
        gyro_offset_[1] = pre_cali_.gyro_offset[1];
        gyro_offset_[2] = pre_cali_.gyro_offset[2];
        g_norm_ = pre_cali_.g_norm;
    }

    // 修正加速度灵敏度
    acc_coef_ *= bmi088::kGNormReference / g_norm_;
}
