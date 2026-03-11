#include "ht_driver.hpp"

#include <cstring>

// ---- 静态成员定义 ----
HtDriver* HtDriver::instances_[MAX_MOTORS]{};
uint8_t HtDriver::instance_count_ = 0;

// ---- 构造函数 ----
HtDriver::HtDriver(const HtDriverConfig& cfg) {
    // 创建 SAL CAN 实例
    sal::CanInstance::CanConfig can_cfg{};
    can_cfg.handle = cfg.can_handle;
    can_cfg.tx_id = cfg.tx_id;
    can_cfg.rx_id = cfg.rx_id;
    can_cfg.rx_cbk = [this](uint8_t /*len*/) {
        DecodeFeedback(can_->RxData());
    };

    can_ = new sal::CanInstance(can_cfg);

    // 注册到静态实例列表
    if (instance_count_ < MAX_MOTORS) {
        instances_[instance_count_++] = this;
    }

    // 使能电机 + 校准编码器
    SendModeCmd(HtModeCmd::MOTOR_MODE);
    CalibrateEncoder();
}

// ---- 纯力矩模式 ----
void HtDriver::SetOutput(float torque) {
    using namespace ht_motor;
    using namespace mit_codec;

    torque = Clamp(torque, T_MIN, T_MAX);

    // HT 电机特殊: 只更新 buf[6:7] 的力矩字段，buf[0:5] 保持校准时的值
    uint16_t t = FloatToUint(torque, T_MIN, T_MAX, 12);
    tx_buf_[6] = static_cast<uint8_t>(t >> 8);
    tx_buf_[7] = static_cast<uint8_t>(t & 0xFF);
    tx_pending_ = true;
}

// ---- 全 MIT 五参数控制 ----
void HtDriver::SetMitOutput(float pos, float vel,
                             float kp, float kd, float torque) {
    using namespace ht_motor;
    using namespace mit_codec;

    pos    = Clamp(pos,    P_MIN, P_MAX);
    vel    = Clamp(vel,    V_MIN, V_MAX);
    kp     = Clamp(kp,    KP_MIN, KP_MAX);
    kd     = Clamp(kd,    KD_MIN, KD_MAX);
    torque = Clamp(torque, T_MIN, T_MAX);

    PackMitFrame(tx_buf_,
                 FloatToUint(pos, P_MIN, P_MAX, 16),
                 FloatToUint(vel, V_MIN, V_MAX, 12),
                 FloatToUint(kp,  KP_MIN, KP_MAX, 12),
                 FloatToUint(kd,  KD_MIN, KD_MAX, 12),
                 FloatToUint(torque, T_MIN, T_MAX, 12));
    tx_pending_ = true;
}

// ---- 模式命令 ----
void HtDriver::SendModeCmd(HtModeCmd cmd) {
    sal::CanMsg msg{};
    mit_codec::PackModeFrame(msg.data, static_cast<uint8_t>(cmd));
    can_->CanTransmit(msg);
}

// ---- 编码器校准 ----
void HtDriver::CalibrateEncoder() {
    using namespace ht_motor;
    using namespace mit_codec;

    // 发送全零 MIT 帧并保存 buf[0:5]（HT 协议要求后续只修改力矩字段）
    PackMitFrame(tx_buf_,
                 FloatToUint(0, P_MIN, P_MAX, 16),
                 FloatToUint(0, V_MIN, V_MAX, 12),
                 FloatToUint(0, KP_MIN, KP_MAX, 12),
                 FloatToUint(0, KD_MIN, KD_MAX, 12),
                 FloatToUint(0, T_MIN, T_MAX, 12));

    sal::CanMsg msg{};
    std::memcpy(msg.data, tx_buf_, 8);
    can_->CanTransmit(msg);

    // 发送零位校准命令
    SendModeCmd(HtModeCmd::ZERO_POSITION);

    // tx_buf_[0:5] 保留校准值，后续 SetOutput 只修改 [6:7]
}

// ---- 批量发送 ----
void HtDriver::FlushAll() {
    for (uint8_t i = 0; i < instance_count_; ++i) {
        auto* drv = instances_[i];
        if (!drv->tx_pending_)
            continue;

        sal::CanMsg msg{};
        std::memcpy(msg.data, drv->tx_buf_, 8);
        drv->can_->CanTransmit(msg);
        drv->tx_pending_ = false;
    }
}

// ---- 滑动均值滤波 ----
float HtDriver::AverageFilter(float new_val) {
    speed_buf_[speed_buf_idx_] = new_val;
    speed_buf_idx_ = (speed_buf_idx_ + 1) % ht_motor::SPEED_BUF_SIZE;

    float sum = 0;
    for (uint8_t i = 0; i < ht_motor::SPEED_BUF_SIZE; ++i)
        sum += speed_buf_[i];
    return sum / static_cast<float>(ht_motor::SPEED_BUF_SIZE);
}

// ---- CAN 反馈解码 (ISR context) ----
void HtDriver::DecodeFeedback(const uint8_t* data) {
    using namespace ht_motor;
    using namespace mit_codec;

    online_cnt_ = 0;

    auto& fb = ht_fb_;

    fb.position = UintToFloat(
        (static_cast<uint16_t>(data[1]) << 8) | data[2],
        P_MIN, P_MAX, 16);

    float raw_vel = UintToFloat(
        (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4),
        V_MIN, V_MAX, 12);
    fb.velocity = AverageFilter(raw_vel - SPEED_BIAS);

    fb.torque = UintToFloat(
        (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5],
        T_MIN, T_MAX, 12);

    // 映射到 MotorMeasure（角度/速度转为 deg 单位）
    auto& m = measure_;
    m.total_angle = fb.position * RAD_2_DEGREE;
    m.speed_aps   = fb.velocity * RAD_2_DEGREE;
}
