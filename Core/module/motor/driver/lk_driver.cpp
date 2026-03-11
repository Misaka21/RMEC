#include "lk_driver.hpp"

#include <cstring>

// ---- 静态成员定义 ----
LkDriver::TxGroup LkDriver::tx_groups_[GROUP_COUNT]{};

// ---- 构造函数 ----
LkDriver::LkDriver(const LkDriverConfig& cfg) {
    // 计算分组和帧内偏移
    uint8_t id = cfg.motor_id;  // 1-based
    msg_offset_ = (id - 1) * 2;

#ifdef CAN2
    extern CAN_HandleTypeDef hcan1;
    group_idx_ = (cfg.can_handle != &hcan1) ? 1 : 0;
#else
    group_idx_ = 0;
#endif

    // 反馈 rx_id = 0x140 + motor_id
    uint32_t rx_id = 0x140 + id;

    // 创建 SAL CAN 实例（tx_id = 0x280 用于批量发送）
    sal::CanInstance::CanConfig can_cfg{};
    can_cfg.handle = cfg.can_handle;
    can_cfg.tx_id = 0x280;
    can_cfg.rx_id = rx_id;
    can_cfg.rx_cbk = [this](uint8_t /*len*/) {
        DecodeFeedback(can_->RxData());
    };

    can_ = new sal::CanInstance(can_cfg);

    // 注册为该组的 sender（第一个注册的电机实例）
    if (tx_groups_[group_idx_].sender == nullptr) {
        tx_groups_[group_idx_].sender = can_;
    }
    tx_groups_[group_idx_].enabled = true;
}

// ---- 设置输出 ----
void LkDriver::SetOutput(float output) {
    auto value = static_cast<int16_t>(Clamp(output,
        static_cast<float>(lk_motor::I_MIN),
        static_cast<float>(lk_motor::I_MAX)));

    auto& group = tx_groups_[group_idx_];

    // LK 多电机帧: 小端序 int16, 与原协议一致
    group.data[msg_offset_]     = static_cast<uint8_t>(value & 0xFF);
    group.data[msg_offset_ + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

// ---- 批量发送 ----
void LkDriver::FlushAll() {
    for (uint8_t i = 0; i < GROUP_COUNT; ++i) {
        auto& group = tx_groups_[i];
        if (!group.enabled || group.sender == nullptr)
            continue;

        sal::CanMsg msg{};
        std::memcpy(msg.data, group.data, 8);
        group.sender->CanTransmit(msg);

        // 清空缓冲区，防止失联后发送旧指令
        std::memset(group.data, 0, 8);
    }
}

// ---- CAN 反馈解码 (ISR context) ----
void LkDriver::DecodeFeedback(const uint8_t* data) {
    using namespace lk_motor;

    online_cnt_ = 0;

    auto& m = measure_;

    m.temperature = data[1];

    // 电流 LPF (int16, big-endian: [3]=high, [2]=low)
    int16_t raw_current = static_cast<int16_t>(
        (static_cast<uint16_t>(data[3]) << 8) | data[2]);
    m.real_current = static_cast<int16_t>(LowPassFilter(
        static_cast<float>(m.real_current),
        static_cast<float>(raw_current),
        CURRENT_SMOOTH_COEF));

    // 速度 LPF (int16 dps → deg/s，与 speed_aps 单位一致)
    int16_t raw_speed = static_cast<int16_t>(
        (static_cast<uint16_t>(data[5]) << 8) | data[4]);
    m.speed_aps = LowPassFilter(
        m.speed_aps,
        static_cast<float>(raw_speed),
        SPEED_SMOOTH_COEF);

    // 编码器 (uint16, 65536 counts)
    m.last_ecd = m.ecd;
    m.ecd = (static_cast<uint16_t>(data[7]) << 8) | data[6];

    // 多圈角度追踪（65536-count 编码器，过零阈值 32768）
    m.angle_single_round = lk_motor::ECD_ANGLE_COEF * static_cast<float>(m.ecd);

    int32_t delta = static_cast<int32_t>(m.ecd) - static_cast<int32_t>(m.last_ecd);
    if (delta > 32768)
        m.total_round--;
    else if (delta < -32768)
        m.total_round++;

    m.total_angle = m.total_round * 360.0f + m.angle_single_round;
}
