#include "referee.hpp"
#include "referee_protocol.hpp"

#include <cstring>

namespace referee {

RefereeParser::RefereeParser(SendFunc send)
    : send_func_(send) {}

// ======================== 字节级状态机 ========================
// DMA IDLE 可能一次给出多帧或半帧, 逐字节推进

void RefereeParser::Parse(const uint8_t* buf, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t byte = buf[i];

        switch (state_) {
        case State::WAIT_SOF:
            if (byte == SOF) {
                rx_buf_[0] = byte;
                rx_idx_ = 1;
                state_ = State::HEADER;
            }
            break;

        case State::HEADER:
            rx_buf_[rx_idx_++] = byte;
            if (rx_idx_ >= HEADER_LEN) {
                // 帧头 CRC8 校验 (前 4 字节 → CRC8 在第 5 字节)
                if (!Crc8Verify(rx_buf_, HEADER_LEN)) {
                    state_ = State::WAIT_SOF;
                    break;
                }

                // 提取 data_length
                data_length_ = static_cast<uint16_t>(rx_buf_[1]) |
                               (static_cast<uint16_t>(rx_buf_[2]) << 8);

                // 整帧长度: header(5) + cmd_id(2) + data(N) + crc16(2)
                frame_len_ = HEADER_LEN + CMD_ID_LEN + data_length_ + CRC16_LEN;

                // 合理性检查
                if (frame_len_ > sizeof(rx_buf_)) {
                    state_ = State::WAIT_SOF;
                    break;
                }

                state_ = State::DATA;
            }
            break;

        case State::DATA:
            rx_buf_[rx_idx_++] = byte;
            if (rx_idx_ >= frame_len_) {
                ProcessFrame();
                state_ = State::WAIT_SOF;
            }
            break;
        }
    }
}

// ======================== 帧处理: CRC16 + 分发 ========================

void RefereeParser::ProcessFrame() {
    // 整帧 CRC16 校验
    if (!Crc16Verify(rx_buf_, frame_len_))
        return;

    // 提取 cmd_id
    uint16_t cmd_id = static_cast<uint16_t>(rx_buf_[HEADER_LEN]) |
                      (static_cast<uint16_t>(rx_buf_[HEADER_LEN + 1]) << 8);

    // 固定长度 cmd_id: 校验 data_length 是否匹配
    uint16_t expected_len = CmdDataLength(cmd_id);
    if (expected_len != 0 && data_length_ != expected_len)
        return;

    // 数据区起始指针
    const uint8_t* data = rx_buf_ + HEADER_LEN + CMD_ID_LEN;

    switch (cmd_id) {
    case CMD_GAME_STATUS: {
        WireGameStatus w{};
        std::memcpy(&w, data, sizeof(w));
        data_.game_status.game_type         = w.game_type_progress & 0x0F;
        data_.game_status.game_progress     = (w.game_type_progress >> 4) & 0x0F;
        data_.game_status.stage_remain_time = w.stage_remain_time;
        data_.game_status.sync_time_stamp   = w.sync_time_stamp;
        break;
    }

    case CMD_GAME_RESULT: {
        WireGameResult w{};
        std::memcpy(&w, data, sizeof(w));
        data_.game_result.winner = w.winner;
        break;
    }

    case CMD_GAME_ROBOT_HP: {
        WireGameRobotHp w{};
        std::memcpy(&w, data, sizeof(w));
        data_.game_robot_hp.red_1_hp        = w.red_1_robot_hp;
        data_.game_robot_hp.red_2_hp        = w.red_2_robot_hp;
        data_.game_robot_hp.red_3_hp        = w.red_3_robot_hp;
        data_.game_robot_hp.red_4_hp        = w.red_4_robot_hp;
        data_.game_robot_hp.red_7_hp        = w.red_7_robot_hp;
        data_.game_robot_hp.red_outpost_hp  = w.red_outpost_hp;
        data_.game_robot_hp.red_base_hp     = w.red_base_hp;
        data_.game_robot_hp.blue_1_hp       = w.blue_1_robot_hp;
        data_.game_robot_hp.blue_2_hp       = w.blue_2_robot_hp;
        data_.game_robot_hp.blue_3_hp       = w.blue_3_robot_hp;
        data_.game_robot_hp.blue_4_hp       = w.blue_4_robot_hp;
        data_.game_robot_hp.blue_7_hp       = w.blue_7_robot_hp;
        data_.game_robot_hp.blue_outpost_hp = w.blue_outpost_hp;
        data_.game_robot_hp.blue_base_hp    = w.blue_base_hp;
        break;
    }

    case CMD_EVENT_DATA: {
        WireEventData w{};
        std::memcpy(&w, data, sizeof(w));
        data_.event_data.event_type = w.event_type;
        break;
    }

    case CMD_REFEREE_WARN: {
        WireRefereeWarning w{};
        std::memcpy(&w, data, sizeof(w));
        data_.referee_warning.level           = w.level;
        data_.referee_warning.offending_robot = w.offending_robot_id;
        data_.referee_warning.count           = w.count;
        break;
    }

    case CMD_DART_INFO: {
        WireDartInfo w{};
        std::memcpy(&w, data, sizeof(w));
        data_.dart_info.dart_remaining_time = w.dart_remaining_time;
        data_.dart_info.dart_info           = w.dart_info;
        break;
    }

    case CMD_ROBOT_STATUS: {
        WireRobotStatus w{};
        std::memcpy(&w, data, sizeof(w));
        data_.robot_status.robot_id                = w.robot_id;
        data_.robot_status.robot_level             = w.robot_level;
        data_.robot_status.current_hp              = w.current_hp;
        data_.robot_status.maximum_hp              = w.maximum_hp;
        data_.robot_status.shooter_barrel_cooling_value = w.shooter_barrel_cooling_value;
        data_.robot_status.shooter_barrel_heat_limit    = w.shooter_barrel_heat_limit;
        data_.robot_status.chassis_power_limit     = w.chassis_power_limit;
        data_.robot_status.power_management_output =
            (w.power_management_gimbal_output) |
            (w.power_management_chassis_output << 1) |
            (w.power_management_shooter_output << 2);
        break;
    }

    case CMD_POWER_HEAT: {
        WirePowerHeat w{};
        std::memcpy(&w, data, sizeof(w));
        data_.power_heat.reserved1                = w.reserved1;
        data_.power_heat.reserved2                = w.reserved2;
        data_.power_heat.reserved3                = w.reserved3;
        data_.power_heat.buffer_energy            = w.buffer_energy;
        data_.power_heat.shooter_17mm_1_barrel_heat = w.shooter_17mm_1_barrel_heat;
        data_.power_heat.shooter_17mm_2_barrel_heat = w.shooter_17mm_2_barrel_heat;
        data_.power_heat.shooter_42mm_barrel_heat   = w.shooter_42mm_barrel_heat;
        break;
    }

    case CMD_ROBOT_POS: {
        WireRobotPos w{};
        std::memcpy(&w, data, sizeof(w));
        data_.robot_pos.x     = w.x;
        data_.robot_pos.y     = w.y;
        data_.robot_pos.angle = w.angle;
        break;
    }

    case CMD_BUFF: {
        WireBuff w{};
        std::memcpy(&w, data, sizeof(w));
        data_.buff.recovery_buff      = w.recovery_buff;
        data_.buff.cooling_buff       = w.cooling_buff;
        data_.buff.defence_buff       = w.defence_buff;
        data_.buff.vulnerability_buff = w.vulnerability_buff;
        data_.buff.attack_buff        = w.attack_buff;
        break;
    }

    case CMD_HURT_DATA: {
        WireHurtData w{};
        std::memcpy(&w, data, sizeof(w));
        data_.hurt_data.armor_id  = w.armor_id_hurt_type & 0x0F;
        data_.hurt_data.hurt_type = (w.armor_id_hurt_type >> 4) & 0x0F;
        break;
    }

    case CMD_SHOOT_DATA: {
        WireShootData w{};
        std::memcpy(&w, data, sizeof(w));
        data_.shoot_data.bullet_type        = w.bullet_type;
        data_.shoot_data.shooter_num        = w.shooter_number;
        data_.shoot_data.launching_frequency = w.launching_frequency;
        data_.shoot_data.initial_speed       = w.initial_speed;
        break;
    }

    case CMD_PROJECTILE_ALW: {
        WireProjectileAllowance w{};
        std::memcpy(&w, data, sizeof(w));
        data_.projectile_allowance.projectile_allowance_17mm = w.projectile_allowance_17mm;
        data_.projectile_allowance.projectile_allowance_42mm = w.projectile_allowance_42mm;
        data_.projectile_allowance.remaining_gold_coin       = w.remaining_gold_coin;
        break;
    }

    case CMD_RFID_STATUS: {
        WireRfidStatus w{};
        std::memcpy(&w, data, sizeof(w));
        data_.rfid_status.rfid_status = w.rfid_status;
        break;
    }

    case CMD_GROUND_POS: {
        WireGroundRobotPosition w{};
        std::memcpy(&w, data, sizeof(w));
        data_.ground_robot_position.hero_x       = w.hero_x;
        data_.ground_robot_position.hero_y       = w.hero_y;
        data_.ground_robot_position.engineer_x   = w.engineer_x;
        data_.ground_robot_position.engineer_y   = w.engineer_y;
        data_.ground_robot_position.standard_3_x = w.standard_3_x;
        data_.ground_robot_position.standard_3_y = w.standard_3_y;
        data_.ground_robot_position.standard_4_x = w.standard_4_x;
        data_.ground_robot_position.standard_4_y = w.standard_4_y;
        data_.ground_robot_position.reserved_x   = w.reserved_x;
        data_.ground_robot_position.reserved_y   = w.reserved_y;
        break;
    }

    case CMD_SENTRY_INFO: {
        WireSentryInfo w{};
        std::memcpy(&w, data, sizeof(w));
        data_.sentry_info.sentry_info = w.sentry_info;
        break;
    }

    case CMD_CUSTOM_CTRL: {
        WireCustomController w{};
        std::memcpy(&w, data, sizeof(w));
        std::memcpy(data_.custom_controller.data, w.data, sizeof(w.data));
        data_.custom_controller.updated = true;
        break;
    }

    default:
        break;
    }
}

// ======================== 发送 ========================

void RefereeParser::Send(uint16_t cmd_id, const uint8_t* payload, uint16_t len) {
    if (!send_func_) return;

    uint16_t frame_len = HEADER_LEN + CMD_ID_LEN + len + CRC16_LEN;
    if (frame_len > sizeof(rx_buf_)) return;  // 复用 rx_buf_ 作为 tx 临时缓冲

    // 帧头
    uint8_t tx_buf[512]{};
    tx_buf[0] = SOF;
    tx_buf[1] = len & 0xFF;
    tx_buf[2] = (len >> 8) & 0xFF;
    tx_buf[3] = 0;  // seq
    tx_buf[4] = Crc8Calc(tx_buf, 4);

    // cmd_id
    tx_buf[HEADER_LEN]     = cmd_id & 0xFF;
    tx_buf[HEADER_LEN + 1] = (cmd_id >> 8) & 0xFF;

    // payload
    if (payload && len > 0) {
        std::memcpy(tx_buf + HEADER_LEN + CMD_ID_LEN, payload, len);
    }

    // CRC16 整帧
    Crc16Append(tx_buf, frame_len);

    send_func_(tx_buf, frame_len);
}

} // namespace referee
