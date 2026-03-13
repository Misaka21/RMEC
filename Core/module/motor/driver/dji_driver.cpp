#include "dji_driver.hpp"
#include "log.h"

#include <cstring>

// ---- 静态成员定义 ----
DjiDriver::TxGroup DjiDriver::tx_groups_[GROUP_COUNT]{};

// ---- 电机类型参数表 (按 DjiMotorType 枚举值索引) ----
namespace {

struct MotorTypeInfo {
    float raw_per_unit;     // 物理量→CAN 原始值换算因子
    float max_physical;     // 物理量上限
    uint8_t min_id;         // 最小合法 motor_id
    uint8_t max_id;         // 最大合法 motor_id
    uint16_t rx_id_base;    // 反馈帧 ID = rx_id_base + motor_id
    uint8_t id_split;       // id <= split → 低组, id > split → 高组
    uint8_t group_lo[2];    // 低组索引 [CAN1, CAN2]
    uint8_t group_hi[2];    // 高组索引 [CAN1, CAN2]
};

using namespace dji_motor;

constexpr MotorTypeInfo TYPE_INFO[] = {
    // M3508: 电流, id 1-8, rx 0x200+id, 低组 0x200, 高组 0x1FF
    {M3508_RAW_PER_AMP, M3508_MAX_CURRENT, 1, 8, 0x200, 4, {0, 3}, {1, 4}},
    // M2006: 电流, id 1-8, rx 0x200+id, 低组 0x200, 高组 0x1FF
    {M2006_RAW_PER_AMP, M2006_MAX_CURRENT, 1, 8, 0x200, 4, {0, 3}, {1, 4}},
    // GM6020: 电压, id 1-7, rx 0x204+id, 低组 0x1FF, 高组 0x2FF
    {GM6020_RAW_PER_VOLT, GM6020_MAX_VOLTAGE, 1, 7, 0x204, 4, {1, 4}, {2, 5}},
    // GM6020_CURRENT: 电流, id 1-7, rx 0x204+id, 低组 0x1FE, 高组 0x2FE
    {GM6020_CURRENT_RAW_PER_AMP, GM6020_CURRENT_MAX, 1, 7, 0x204, 4, {6, 8}, {7, 9}},
};

static_assert(sizeof(TYPE_INFO) / sizeof(TYPE_INFO[0]) == static_cast<uint8_t>(DjiMotorType::GM6020_CURRENT) + 1,
              "TYPE_INFO must cover all DjiMotorType values");

} // namespace

#ifdef CAN2
extern CAN_HandleTypeDef hcan1;
#endif

namespace {

bool IsCan2([[maybe_unused]] CAN_HandleTypeDef* handle) {
#ifdef CAN2
    return handle != &hcan1;
#else
    return false;
#endif
}

} // namespace

// ---- 构造函数 ----
DjiDriver::DjiDriver(const DjiDriverConfig& cfg)
    : motor_type_(cfg.motor_type)
{
    const auto& info = TYPE_INFO[static_cast<uint8_t>(motor_type_)];
    uint8_t id = cfg.motor_id;

    // ID 范围校验
    if (id < info.min_id || id > info.max_id)
        DEBUG_DEADLOCK("[DjiDriver] motor_id out of valid range");

    // 物理量换算参数
    physical_to_raw_ = info.raw_per_unit;
    max_physical_    = info.max_physical;

    // 分组计算
    uint8_t can_idx = IsCan2(cfg.can_handle) ? 1 : 0;
    if (id <= info.id_split) {
        group_idx_  = info.group_lo[can_idx];
        msg_offset_ = (id - info.min_id) * 2;
    } else {
        group_idx_  = info.group_hi[can_idx];
        msg_offset_ = (id - info.id_split - 1) * 2;
    }

    // RX ID 计算 + 同总线重复检测
    uint32_t rx_id = info.rx_id_base + id;

    static uint32_t registered_rx[2][MAX_MOTORS] = {};
    static uint8_t registered_cnt[2] = {};
    for (uint8_t i = 0; i < registered_cnt[can_idx]; ++i) {
        if (registered_rx[can_idx][i] == rx_id)
            DEBUG_DEADLOCK("[DjiDriver] duplicate rx_id on same CAN bus");
    }
    if (registered_cnt[can_idx] < MAX_MOTORS)
        registered_rx[can_idx][registered_cnt[can_idx]++] = rx_id;

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

// ---- 设置输出 (物理量 → CAN 原始值) ----
void DjiDriver::SetOutput(float output) {
    output = Clamp(output, -max_physical_, max_physical_);
    auto value = static_cast<int16_t>(output * physical_to_raw_);
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
