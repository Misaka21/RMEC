#include "dm_driver.hpp"

#include <cstring>

// ---- 静态成员定义 ----
DmDriver* DmDriver::instances_[MAX_MOTORS]{};
uint8_t DmDriver::instance_count_ = 0;

// ---- 构造函数 ----
DmDriver::DmDriver(const DmDriverConfig& cfg) {
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

    // 使能电机并校准编码器零位
    SendModeCmd(DmModeCmd::MOTOR_MODE);
    SendModeCmd(DmModeCmd::ZERO_POSITION);
}

// ---- 纯力矩模式 ----
void DmDriver::SetOutput(float torque) {
    SetMitOutput(0, 0, 0, 0, torque);
}

// ---- 全 MIT 五参数控制 ----
void DmDriver::SetMitOutput(float pos, float vel,
                             float kp, float kd, float torque) {
    using namespace dm_motor;
    using namespace mit_codec;

    torque = Clamp(torque, T_MIN, T_MAX);
    pos    = Clamp(pos,    P_MIN, P_MAX);
    vel    = Clamp(vel,    V_MIN, V_MAX);
    kp     = Clamp(kp,    KP_MIN, KP_MAX);
    kd     = Clamp(kd,    KD_MIN, KD_MAX);

    PackMitFrame(tx_buf_,
                 FloatToUint(pos, P_MIN, P_MAX, 16),
                 FloatToUint(vel, V_MIN, V_MAX, 12),
                 FloatToUint(kp,  KP_MIN, KP_MAX, 12),
                 FloatToUint(kd,  KD_MIN, KD_MAX, 12),
                 FloatToUint(torque, T_MIN, T_MAX, 12));
    tx_pending_ = true;
}

// ---- 模式命令 ----
void DmDriver::SendModeCmd(DmModeCmd cmd) {
    sal::CanMsg msg{};
    mit_codec::PackModeFrame(msg.data, static_cast<uint8_t>(cmd));
    can_->CanTransmit(msg);
}

// ---- 批量发送 ----
void DmDriver::FlushAll() {
    for (uint8_t i = 0; i < instance_count_; ++i) {
        auto* drv = instances_[i];
        if (!drv->tx_pending_)
            continue;

        sal::CanMsg msg{};
        std::memcpy(msg.data, drv->tx_buf_, 8);
        drv->can_->CanTransmit(msg);
        drv->tx_pending_ = false;

        // 清空缓冲区，防止失联后发送旧指令
        std::memset(drv->tx_buf_, 0, 8);
    }
}

// ---- CAN 反馈解码 (ISR context) ----
void DmDriver::DecodeFeedback(const uint8_t* data) {
    using namespace dm_motor;
    using namespace mit_codec;

    online_cnt_ = 0;

    auto& fb = dm_fb_;
    fb.position = UintToFloat(
        (static_cast<uint16_t>(data[1]) << 8) | data[2],
        P_MIN, P_MAX, 16);

    fb.velocity = UintToFloat(
        (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4),
        V_MIN, V_MAX, 12);

    fb.torque = UintToFloat(
        (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5],
        T_MIN, T_MAX, 12);

    fb.t_mos   = static_cast<float>(data[6]);
    fb.t_rotor = static_cast<float>(data[7]);

    // 映射到 MotorMeasure（角度/速度转为 deg 单位）
    auto& m = measure_;
    m.total_angle = fb.position * RAD_2_DEGREE;
    m.speed_aps   = fb.velocity * RAD_2_DEGREE;
}
