#include "dji_driver.hpp"
#include "log.h"

#include <cstring>

// ---- 静态成员定义 ----
DjiDriver::TxGroup DjiDriver::tx_groups_[GROUP_COUNT]{};

// ---- 构造函数 ----
DjiDriver::DjiDriver(const DjiDriverConfig& cfg)
    : motor_type_(cfg.motor_type)
{
    // 计算分组
    SetupGrouping(cfg);

    // 计算 RX ID
    uint32_t rx_id = 0;
    switch (motor_type_) {
    case DjiMotorType::M3508:
    case DjiMotorType::M2006:
        rx_id = 0x200 + cfg.motor_id;
        break;
    case DjiMotorType::GM6020:
        rx_id = 0x204 + cfg.motor_id;
        break;
    }

    // 创建 SAL CAN 实例（生命周期由 SAL 静态 vector 管理）
    // tx_id 设为本组的 StdId，用于 FlushAll 时复用此实例发送
    sal::CanInstance::CanConfig can_cfg{};
    can_cfg.handle = cfg.can_handle;
    can_cfg.tx_id = GROUP_STD_IDS[group_idx_];
    can_cfg.rx_id = rx_id;
    can_cfg.rx_cbk = [this](uint8_t /*len*/) {
        DecodeFeedback(can_->RxData());
    };

    can_ = new sal::CanInstance(can_cfg);

    // 注册为该组的 sender（第一个注册的电机的 CAN 实例）
    if (tx_groups_[group_idx_].sender == nullptr) {
        tx_groups_[group_idx_].sender = can_;
    }
    tx_groups_[group_idx_].enabled = true;
}

// ---- 分组计算 ----
void DjiDriver::SetupGrouping(const DjiDriverConfig& cfg) {
    uint8_t id = cfg.motor_id;  // 1-based
    bool is_can2 = false;

#ifdef CAN2
    extern CAN_HandleTypeDef hcan1;
    is_can2 = (cfg.can_handle != &hcan1);
#endif

    uint8_t base = is_can2 ? 3 : 0;  // CAN2 组偏移

    switch (cfg.motor_type) {
    case DjiMotorType::M3508:
    case DjiMotorType::M2006:
        if (id >= 1 && id <= 4) {
            group_idx_ = base + 0;           // 0x200 组
            msg_offset_ = (id - 1) * 2;     // 帧内字节偏移
        } else {
            group_idx_ = base + 1;           // 0x1FF 组
            msg_offset_ = (id - 5) * 2;
        }
        break;

    case DjiMotorType::GM6020:
        if (id >= 1 && id <= 4) {
            group_idx_ = base + 1;           // 0x1FF 组
            msg_offset_ = (id - 1) * 2;
        } else {
            group_idx_ = base + 2;           // 0x2FF 组
            msg_offset_ = (id - 5) * 2;
        }
        break;
    }
}

// ---- 设置输出 ----
void DjiDriver::SetOutput(float output) {
    auto value = static_cast<int16_t>(output);
    auto& group = tx_groups_[group_idx_];
    group.data[msg_offset_]     = static_cast<uint8_t>(value >> 8);
    group.data[msg_offset_ + 1] = static_cast<uint8_t>(value & 0xFF);
}

// ---- 批量发送 ----
void DjiDriver::FlushAll() {
    for (uint8_t i = 0; i < GROUP_COUNT; ++i) {
        auto& group = tx_groups_[i];
        if (!group.enabled || group.sender == nullptr)
            continue;

        sal::CanMsg msg{};
        std::memcpy(msg.data, group.data, 8);
        group.sender->CanTransmit(msg);

        // 清空缓冲区，防止电机失联后仍发送旧指令
        std::memset(group.data, 0, 8);
    }
}

// ---- CAN 反馈解码 (ISR context) ----
void DjiDriver::DecodeFeedback(const uint8_t* data) {
    // 收到反馈，重置离线计数
    online_cnt_ = 0;

    auto& m = measure_;
    m.last_ecd = m.ecd;
    m.ecd = (static_cast<uint16_t>(data[0]) << 8) | data[1];

    // 速度 LPF (rpm → deg/s)
    int16_t raw_speed = static_cast<int16_t>(
        (static_cast<uint16_t>(data[2]) << 8) | data[3]);
    m.speed_aps = LowPassFilter(
        m.speed_aps,
        RPM_2_ANGLE_PER_SEC * static_cast<float>(raw_speed),
        SPEED_SMOOTH_COEF);

    // 电流 LPF
    int16_t raw_current = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) | data[5]);
    m.real_current = static_cast<int16_t>(LowPassFilter(
        static_cast<float>(m.real_current),
        static_cast<float>(raw_current),
        CURRENT_SMOOTH_COEF));

    m.temperature = data[6];

    // 多圈角度追踪
    m.UpdateAngle(ECD_ANGLE_COEF);
}
