#pragma once

#include <cstdint>
#include <cstring>

namespace referee {

// ======================== 帧格式常量 ========================

inline constexpr uint8_t  SOF         = 0xA5;  ///< 帧头起始字节
inline constexpr uint16_t HEADER_LEN  = 5;     ///< 帧头长度: SOF(1) + data_length(2) + seq(1) + CRC8(1)
inline constexpr uint16_t CMD_ID_LEN  = 2;     ///< 命令码长度
inline constexpr uint16_t CRC16_LEN   = 2;     ///< 帧尾 CRC16 长度
inline constexpr uint16_t MAX_DATA_LEN = 300;  ///< 最大 data 段长度 (0x0310 自定义客户端)
inline constexpr uint16_t MAX_FRAME_LEN = HEADER_LEN + CMD_ID_LEN + MAX_DATA_LEN + CRC16_LEN;

// ======================== CMD ID 定义 ========================
// 格式: 服务器/主控→机器人 (接收), 机器人→服务器/选手端 (发送)

inline constexpr uint16_t CMD_GAME_STATUS     = 0x0001;  ///< 比赛状态 (11B, 1Hz)
inline constexpr uint16_t CMD_GAME_RESULT     = 0x0002;  ///< 比赛结果 (1B, 结束触发)
inline constexpr uint16_t CMD_GAME_ROBOT_HP   = 0x0003;  ///< 己方血量 (16B, 3Hz, V1.2.0 仅己方)
inline constexpr uint16_t CMD_EVENT_DATA      = 0x0101;  ///< 场地事件 (4B, 1Hz)
inline constexpr uint16_t CMD_REFEREE_WARN    = 0x0104;  ///< 裁判警告 (3B, 判罚触发/1Hz)
inline constexpr uint16_t CMD_DART_INFO       = 0x0105;  ///< 飞镖信息 (3B, 1Hz)
inline constexpr uint16_t CMD_ROBOT_STATUS    = 0x0201;  ///< 机器人状态 (13B, 10Hz)
inline constexpr uint16_t CMD_POWER_HEAT      = 0x0202;  ///< 功率热量 (14B, 10Hz)
inline constexpr uint16_t CMD_ROBOT_POS       = 0x0203;  ///< 机器人位置 (16B, 1Hz)
inline constexpr uint16_t CMD_BUFF            = 0x0204;  ///< 增益 (8B, 3Hz)
inline constexpr uint16_t CMD_HURT_DATA       = 0x0206;  ///< 伤害状态 (1B, 受伤触发)
inline constexpr uint16_t CMD_SHOOT_DATA      = 0x0207;  ///< 射击信息 (7B, 发射触发)
inline constexpr uint16_t CMD_PROJECTILE_ALW  = 0x0208;  ///< 允许发弹量 (8B, 10Hz)
inline constexpr uint16_t CMD_RFID_STATUS     = 0x0209;  ///< RFID 状态 (5B, 3Hz)
inline constexpr uint16_t CMD_GROUND_POS      = 0x020B;  ///< 全场位置 (40B, 1Hz, 哨兵专用)
inline constexpr uint16_t CMD_SENTRY_INFO     = 0x020D;  ///< 哨兵信息同步 (6B, 1Hz)
inline constexpr uint16_t CMD_INTERACTION     = 0x0301;  ///< 机器人交互数据 (<=118B, 30Hz 上限)
inline constexpr uint16_t CMD_CUSTOM_CTRL     = 0x0302;  ///< 自定义控制器 (30B, 30Hz, 图传链路)
inline constexpr uint16_t CMD_MAP_COMMAND     = 0x0303;  ///< 小地图交互 (15B, 选手端触发)
inline constexpr uint16_t CMD_CUSTOM_CLIENT   = 0x0310;  ///< 自定义客户端 (<=300B, 50Hz, 图传链路)
inline constexpr uint16_t CMD_CLIENT_CMD      = 0x0311;  ///< 自定义客户端指令 (12B, 75Hz, 图传链路)

// ======================== 0x0301 交互数据子命令 ========================
// 机器人交互数据内容数据段最大 112B (帧总长 127B - 9B 帧开销 - 6B 数据段头)
// 整个 0x0301 上行频率最大 30Hz, 需合理安排带宽

inline constexpr uint16_t SUB_CMD_DELETE_LAYER  = 0x0100;  ///< 删除图层 (2B)
inline constexpr uint16_t SUB_CMD_DRAW_1        = 0x0101;  ///< 绘制 1 个图形 (15B)
inline constexpr uint16_t SUB_CMD_DRAW_2        = 0x0102;  ///< 绘制 2 个图形 (30B)
inline constexpr uint16_t SUB_CMD_DRAW_5        = 0x0103;  ///< 绘制 5 个图形 (75B)
inline constexpr uint16_t SUB_CMD_DRAW_7        = 0x0104;  ///< 绘制 7 个图形 (105B)
inline constexpr uint16_t SUB_CMD_DRAW_STRING   = 0x0110;  ///< 绘制字符串 (45B = 15B 配置 + 30B 字符)
inline constexpr uint16_t SUB_CMD_SENTRY_CMD    = 0x0120;  ///< 哨兵自主决策指令 (4B)
inline constexpr uint16_t SUB_CMD_RADAR_CMD     = 0x0121;  ///< 雷达自主决策指令 (1B)

// ======================== 各 cmd_id 的 data_length ========================
// 返回 0 表示可变长度或未知 cmd_id (跳过长度校验)

inline constexpr uint16_t CmdDataLength(uint16_t cmd_id) {
    switch (cmd_id) {
    case CMD_GAME_STATUS:    return 11;
    case CMD_GAME_RESULT:    return 1;
    case CMD_GAME_ROBOT_HP:  return 16;
    case CMD_EVENT_DATA:     return 4;
    case CMD_REFEREE_WARN:   return 3;
    case CMD_DART_INFO:      return 3;
    case CMD_ROBOT_STATUS:   return 13;
    case CMD_POWER_HEAT:     return 14;
    case CMD_ROBOT_POS:      return 16;
    case CMD_BUFF:           return 8;
    case CMD_HURT_DATA:      return 1;
    case CMD_SHOOT_DATA:     return 7;
    case CMD_PROJECTILE_ALW: return 8;
    case CMD_RFID_STATUS:    return 5;
    case CMD_GROUND_POS:     return 40;
    case CMD_SENTRY_INFO:    return 6;
    case CMD_CUSTOM_CTRL:    return 30;
    case CMD_MAP_COMMAND:    return 15;
    case CMD_CLIENT_CMD:     return 12;
    default:                 return 0;  // 可变长或未知
    }
}

// ======================== CRC8 (帧头 5B 校验) ========================
// 多项式: x^8 + x^5 + x^4 + 1 (0x31), 初始值 0xFF

inline constexpr uint8_t CRC8_INIT = 0xFF;

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays)
inline constexpr uint8_t CRC8_TABLE[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};
// NOLINTEND(cppcoreguidelines-avoid-c-arrays)

inline uint8_t Crc8Calc(const uint8_t* data, uint16_t len) {
    uint8_t crc = CRC8_INIT;
    while (len-- != 0U) {
        crc = CRC8_TABLE[crc ^ *data++];
    }
    return crc;
}

inline bool Crc8Verify(const uint8_t* data, uint16_t len) {
    if (len < 1) return false;
    return Crc8Calc(data, len - 1) == data[len - 1];
}

// ======================== CRC16 (整帧校验) ========================
// 多项式: x^16 + x^15 + x^2 + 1 (0x8005), 初始值 0xFFFF

inline constexpr uint16_t CRC16_INIT = 0xFFFF;

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays)
inline constexpr uint16_t CRC16_TABLE[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};
// NOLINTEND(cppcoreguidelines-avoid-c-arrays)

inline uint16_t Crc16Calc(const uint8_t* data, uint16_t len) {
    uint16_t crc = CRC16_INIT;
    while (len-- != 0U) {
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ *data++) & 0xFF];
    }
    return crc;
}

inline bool Crc16Verify(const uint8_t* data, uint16_t len) {
    if (len <= 2) return false;
    uint16_t expected = Crc16Calc(data, len - 2);
    uint16_t received = static_cast<uint16_t>(data[len - 2]) |
                        (static_cast<uint16_t>(data[len - 1]) << 8);
    return expected == received;
}

inline void Crc16Append(uint8_t* data, uint16_t len) {
    if (len <= 2) return;
    uint16_t crc = Crc16Calc(data, len - 2);
    data[len - 2] = crc & 0xFF;
    data[len - 1] = (crc >> 8) & 0xFF;
}

// ======================== 协议原始 packed 结构体 ========================
// 仅在 ProcessFrame 内部 memcpy 使用, 与 V1.2.0 字节对齐
// @warning 这些结构体仅用于协议解析, 不要直接作为消费端数据结构

#pragma pack(push, 1)

/// 帧头 (5B)
struct FrameHeader {
    uint8_t  sof;          ///< 起始字节, 固定 0xA5
    uint16_t data_length;  ///< data 段长度
    uint8_t  seq;          ///< 包序号
    uint8_t  crc8;         ///< 帧头 CRC8
};

/// 0x0001 比赛状态 (11B)
struct WireGameStatus {
    uint8_t  game_type_progress;  ///< bit 0-3: 比赛类型, bit 4-7: 比赛阶段
    uint16_t stage_remain_time;   ///< 当前阶段剩余时间 (s)
    uint64_t sync_time_stamp;     ///< UNIX 时间戳
};

/// 0x0002 比赛结果 (1B)
struct WireGameResult {
    uint8_t winner;  ///< 0=平局, 1=红方胜利, 2=蓝方胜利
};

/// 0x0003 己方机器人血量 (16B, V1.2.0 仅己方)
struct WireGameRobotHp {
    uint16_t ally_1_robot_hp;  ///< 己方英雄
    uint16_t ally_2_robot_hp;  ///< 己方工程
    uint16_t ally_3_robot_hp;  ///< 己方步兵 3
    uint16_t ally_4_robot_hp;  ///< 己方步兵 4
    uint16_t reserved;         ///< 保留
    uint16_t ally_7_robot_hp;  ///< 己方哨兵
    uint16_t ally_outpost_hp;  ///< 己方前哨站
    uint16_t ally_base_hp;     ///< 己方基地
};

/// 0x0101 场地事件 (4B)
struct WireEventData {
    uint32_t event_type;  ///< bit 编码, 详见 EventData 注释
};

/// 0x0104 裁判警告 (3B)
struct WireRefereeWarning {
    uint8_t level;               ///< 1=双方黄牌, 2=黄牌, 3=红牌, 4=判负
    uint8_t offending_robot_id;  ///< 犯规机器人 ID (判负/双方黄牌时为 0)
    uint8_t count;               ///< 违规次数
};

/// 0x0105 飞镖信息 (3B)
struct WireDartInfo {
    uint8_t  dart_remaining_time;  ///< 飞镖发射剩余时间 (s)
    uint16_t dart_info;            ///< bit 编码, 详见 DartInfo 注释
};

/// 0x0201 机器人状态 (13B)
struct WireRobotStatus {
    uint8_t  robot_id;                       ///< 红方 1-11, 蓝方 101-111
    uint8_t  robot_level;                    ///< 等级 1-3
    uint16_t current_hp;                     ///< 当前血量
    uint16_t maximum_hp;                     ///< 血量上限
    uint16_t shooter_barrel_cooling_value;   ///< 射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;      ///< 射击热量上限
    uint16_t chassis_power_limit;            ///< 底盘功率上限 (W)
    uint8_t  power_management_gimbal_output  : 1;  ///< gimbal 口 (0=无, 1=24V)
    uint8_t  power_management_chassis_output : 1;  ///< chassis 口 (0=无, 1=24V)
    uint8_t  power_management_shooter_output : 1;  ///< shooter 口 (0=无, 1=24V)
};

/// 0x0202 功率热量 (14B, V1.2.0)
/// @note 前 8B 保留, 不再包含 chassis_power
struct WirePowerHeat {
    uint16_t reserved1;                    ///< 保留
    uint16_t reserved2;                    ///< 保留
    float    reserved3;                    ///< 保留 (4B float, 非 uint16_t)
    uint16_t buffer_energy;                ///< 缓冲能量 (J)
    uint16_t shooter_17mm_barrel_heat;     ///< 17mm 射击热量
    uint16_t shooter_42mm_barrel_heat;     ///< 42mm 射击热量
};

/// 0x0203 机器人位置 (16B)
struct WireRobotPos {
    float x;         ///< x 坐标 (m)
    float y;         ///< y 坐标 (m)
    float angle;     ///< 测速模块朝向 (度, 正北为 0°)
    float reserved;  ///< 保留 (凑齐 16B)
};

/// 0x0204 增益 (8B)
struct WireBuff {
    uint8_t  recovery_buff;       ///< 回血增益 (百分比)
    uint16_t cooling_buff;        ///< 冷却增益 (直接值, uint16_t)
    uint8_t  defence_buff;        ///< 防御增益 (百分比)
    uint8_t  vulnerability_buff;  ///< 负防御增益 (百分比)
    uint16_t attack_buff;         ///< 攻击增益 (百分比)
    uint8_t  remaining_energy;    ///< 剩余能量反馈 (bit 编码), 详见 Buff 注释
};

/// 0x0206 伤害状态 (1B)
struct WireHurtData {
    uint8_t armor_id_hurt_type;  ///< bit 0-3: 装甲板 ID, bit 4-7: 伤害类型
};

/// 0x0207 射击信息 (7B)
struct WireShootData {
    uint8_t bullet_type;          ///< 1=17mm, 2=42mm
    uint8_t shooter_number;       ///< 1=17mm 机构, 3=42mm 机构
    uint8_t launching_frequency;  ///< 射频 (Hz)
    float   initial_speed;        ///< 初速度 (m/s)
};

/// 0x0208 允许发弹量 (8B)
struct WireProjectileAllowance {
    uint16_t projectile_allowance_17mm;      ///< 自身 17mm 允许发弹量
    uint16_t projectile_allowance_42mm;      ///< 42mm 允许发弹量
    uint16_t remaining_gold_coin;            ///< 剩余金币
    uint16_t projectile_allowance_fortress;  ///< 堡垒储备 17mm 允许发弹量
};

/// 0x0209 RFID 状态 (5B)
struct WireRfidStatus {
    uint32_t rfid_status;    ///< bit 编码, 详见 RfidStatus 注释
    uint8_t  rfid_status_2;  ///< 隧道对方侧 RFID 点位
};

/// 0x020B 全场地面机器人位置 (40B, 哨兵专用)
struct WireGroundRobotPosition {
    float hero_x;        ///< 己方英雄 x (m)
    float hero_y;        ///< 己方英雄 y (m)
    float engineer_x;    ///< 己方工程 x (m)
    float engineer_y;    ///< 己方工程 y (m)
    float standard_3_x;  ///< 己方步兵 3 x (m)
    float standard_3_y;  ///< 己方步兵 3 y (m)
    float standard_4_x;  ///< 己方步兵 4 x (m)
    float standard_4_y;  ///< 己方步兵 4 y (m)
    float reserved_x;    ///< 保留
    float reserved_y;    ///< 保留
};

/// 0x020D 哨兵信息同步 (6B)
struct WireSentryInfo {
    uint32_t sentry_info;    ///< bit 编码, 详见 SentryInfo 注释
    uint16_t sentry_info_2;  ///< bit 编码, 详见 SentryInfo::sentry_info_2
};

/// 0x0302 自定义控制器 (30B, 图传链路)
struct WireCustomController {
    uint8_t data[30];  ///< 30B 自定义数据
};

// ======================== 交互数据 (0x0301) 发送用 ========================

/// 交互数据头 (6B): 所有 0x0301 子命令共用
struct WireInteractionHeader {
    uint16_t sub_cmd_id;   ///< 子内容 ID (0x0100~0x02FF)
    uint16_t sender_id;    ///< 发送者机器人 ID (需与自身匹配)
    uint16_t receiver_id;  ///< 接收者 ID (仅限己方通信, 选手端=robot_id+0x0100)
};

/// 图形删除操作 (2B): sub_cmd_id = 0x0100
struct WireGraphicDelete {
    /// - 0: 空操作
    /// - 1: 删除指定图层
    /// - 2: 删除所有图层
    uint8_t delete_type;

    /// 图层号 0~9
    uint8_t layer;
};

/// 图形元素 (15B): sub_cmd_id = 0x0101~0x0104
/// @note 屏幕坐标: (0,0)=左下角, (1920,1080)=右上角
struct WireGraphicData {
    uint8_t  figure_name[3];          ///< 图形名 (3B, 用于删除/修改索引)
    uint32_t operate_type : 3;        ///< 操作: 0=空, 1=增加, 2=修改, 3=删除
    uint32_t figure_type  : 3;        ///< 类型: 0=直线, 1=矩形, 2=正圆, 3=椭圆, 4=圆弧, 5=浮点, 6=整型, 7=字符
    uint32_t layer        : 4;        ///< 图层 0~9
    uint32_t color        : 4;        ///< 颜色: 0=己方色, 1=黄, 2=绿, 3=橙, 4=紫红, 5=粉, 6=青, 7=黑, 8=白
    uint32_t details_a    : 9;        ///< 圆弧起始角度 / 浮点字体大小 / 整型字体大小 / 字符字体大小
    uint32_t details_b    : 9;        ///< 圆弧终止角度 / 字符长度(<=30)
    uint32_t width        : 10;       ///< 线宽 (建议字体:线宽 = 10:1)
    uint32_t start_x      : 11;       ///< 起点/圆心 x
    uint32_t start_y      : 11;       ///< 起点/圆心 y
    uint32_t details_c    : 10;       ///< 圆半径 / 浮点数低10bit / 整型低10bit
    uint32_t details_d    : 11;       ///< 终点x / 椭圆rx / 浮点数中11bit / 整型中11bit
    uint32_t details_e    : 11;       ///< 终点y / 椭圆ry / 浮点数高11bit / 整型高11bit
};

/// 哨兵自主决策指令 (4B): sub_cmd_id = 0x0120, receiver_id = 0x8080 (服务器)
struct WireSentryCmd {
    /// bit 编码:
    /// - bit 0: 确认复活 (0=不复活, 1=确认)
    /// - bit 1: 确认兑换立即复活 (0=不兑换, 1=兑换)
    /// - bit 2-12: 将要兑换的发弹量值 (需单调递增)
    /// - bit 13-16: 远程兑换发弹量请求次数 (需单调递增, 每次+1)
    /// - bit 17-20: 远程兑换血量请求次数 (需单调递增, 每次+1)
    /// - bit 21-22: 修改姿态 (1=进攻, 2=防御, 3=移动, 默认 3)
    /// - bit 23: 确认使能量机关进入激活状态 (1=确认)
    uint32_t sentry_cmd;
};

/// 选手端小地图接收机器人消息 (34B): cmd_id = 0x0308
struct WireCustomInfo {
    uint16_t sender_id;       ///< 发送者机器人 ID
    uint16_t receiver_id;     ///< 接收者选手端 ID
    uint8_t  user_data[30];   ///< UTF-16 编码字符数据
};

/// 小地图路径 (103B): cmd_id = 0x0307, 频率上限 1Hz
struct WireMapPath {
    uint8_t  intention;       ///< 1=攻击, 2=防守, 3=移动
    uint16_t start_x;         ///< 起始 x, 单位: dm
    uint16_t start_y;         ///< 起始 y, 单位: dm
    int8_t   delta_x[49];    ///< 路径增量 x
    int8_t   delta_y[49];    ///< 路径增量 y
    uint16_t sender_id;       ///< 发送者机器人 ID
};

#pragma pack(pop)

} // namespace referee
