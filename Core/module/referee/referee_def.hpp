#pragma once

#include <cstdint>
#include <type_traits>

namespace referee {

/// @brief 比赛状态 (cmd_id: 0x0001, 1Hz, 服务器→全体机器人)
struct GameStatus {
    /// 比赛类型 (原始 bit 0-3)
    /// - 1: RoboMaster 机甲大师超级对抗赛
    /// - 2: RoboMaster 机甲大师高校单项赛
    /// - 3: ICRA RoboMaster 高校人工智能挑战赛
    /// - 4: RoboMaster 机甲大师高校联盟赛 3V3 对抗
    /// - 5: RoboMaster 机甲大师高校联盟赛步兵对抗
    uint8_t  game_type          = 0;

    /// 当前比赛阶段 (原始 bit 4-7)
    /// - 0: 未开始比赛
    /// - 1: 准备阶段
    /// - 2: 十五秒裁判系统自检阶段
    /// - 3: 五秒倒计时
    /// - 4: 比赛中
    /// - 5: 比赛结算中
    uint8_t  game_progress      = 0;

    /// 当前阶段剩余时间, 单位: 秒
    uint16_t stage_remain_time  = 0;

    /// UNIX 时间戳, 当机器人正确连接到裁判系统 NTP 服务器后生效
    uint64_t sync_time_stamp    = 0;
};

/// @brief 比赛结果 (cmd_id: 0x0002, 比赛结束后触发)
struct GameResult {
    /// - 0: 平局
    /// - 1: 红方胜利
    /// - 2: 蓝方胜利
    uint8_t winner = 0;
};

/// @brief 己方机器人血量 (cmd_id: 0x0003, 3Hz, V1.2.0 仅己方 16B)
/// 若机器人未上场或被罚下, 则血量为 0
struct GameRobotHp {
    uint16_t ally_1_hp  = 0;  ///< 己方 1 号英雄机器人血量
    uint16_t ally_2_hp  = 0;  ///< 己方 2 号工程机器人血量
    uint16_t ally_3_hp  = 0;  ///< 己方 3 号步兵机器人血量
    uint16_t ally_4_hp  = 0;  ///< 己方 4 号步兵机器人血量
    uint16_t ally_7_hp  = 0;  ///< 己方 7 号哨兵机器人血量
    uint16_t ally_outpost_hp = 0;  ///< 己方前哨站血量
    uint16_t ally_base_hp    = 0;  ///< 己方基地血量
};

/// @brief 场地事件 (cmd_id: 0x0101, 1Hz, 服务器→己方全体机器人)
/// 所有 bit 中, 0=未占领/未激活, 1=已占领/已激活 (除非另有说明)
struct EventData {
    /// bit 编码的场地事件:
    /// - bit 0: 己方与资源区不重叠的补给区占领状态
    /// - bit 1: 己方与资源区重叠的补给区占领状态
    /// - bit 2: 己方补给区占领状态 (仅 RMUL)
    /// - bit 3-4: 己方小能量机关激活状态 (0=未激活, 1=已激活, 2=正在激活)
    /// - bit 5-6: 己方大能量机关激活状态 (0=未激活, 1=已激活, 2=正在激活)
    /// - bit 7-8: 己方中央高地占领状态 (1=己方占领, 2=对方占领)
    /// - bit 9-10: 己方梯形高地占领状态 (1=已占领)
    /// - bit 11-19: 对方飞镖最后一次击中己方前哨站/基地的时间 (0-420s, 开局=0)
    /// - bit 20-22: 对方飞镖最后击中目标 (0=无, 1=前哨站, 2=基地固定, 3=基地随机固定, 4=基地随机移动, 5=基地末端移动)
    /// - bit 23-24: 中心增益点占领状态 (0=未占领, 1=己方, 2=对方, 3=双方) (仅 RMUL)
    /// - bit 25-26: 己方堡垒增益点占领状态 (0=未占领, 1=己方, 2=对方, 3=双方)
    /// - bit 27-28: 己方前哨站增益点占领状态 (0=未占领, 1=己方, 2=对方)
    /// - bit 29: 己方基地增益点占领状态 (1=已占领)
    uint32_t event_type = 0;
};

/// @brief 裁判警告 (cmd_id: 0x0104, 判罚时触发, 其余 1Hz)
struct RefereeWarning {
    /// 己方最后一次受到判罚的等级:
    /// - 1: 双方黄牌
    /// - 2: 黄牌
    /// - 3: 红牌
    /// - 4: 判负
    uint8_t  level           = 0;

    /// 犯规机器人 ID (判负和双方黄牌时为 0)
    /// @see robot_id 编号: 红方 1-11, 蓝方 101-111
    uint8_t  offending_robot = 0;

    /// 该机器人对应判罚等级的违规次数 (开局默认 0)
    uint8_t  count           = 0;
};

/// @brief 飞镖信息 (cmd_id: 0x0105, 1Hz, 服务器→己方全体机器人)
struct DartInfo {
    /// 己方飞镖发射剩余时间, 单位: 秒
    uint8_t  dart_remaining_time = 0;

    /// bit 编码:
    /// - bit 0-2: 最近一次己方飞镖击中目标 (0=无, 1=前哨站, 2=基地固定, 3=基地随机固定, 4=基地随机移动, 5=基地末端移动)
    /// - bit 3-5: 对方最近被击中目标累计计次 (0-4)
    /// - bit 6-8: 飞镖此时选定击打目标 (0=未选定/前哨站, 1=基地固定, 2=基地随机固定, 3=基地随机移动, 4=基地末端移动)
    uint16_t dart_info           = 0;
};

/// @brief 机器人状态 (cmd_id: 0x0201, 10Hz, 主控模块→对应机器人)
struct RobotStatus {
    /// 本机器人 ID
    /// - 红方: 1=英雄, 2=工程, 3/4/5=步兵, 6=空中, 7=哨兵, 8=飞镖, 9=雷达
    /// - 蓝方: 101=英雄, 102=工程, 103/104/105=步兵, 106=空中, 107=哨兵, 108=飞镖, 109=雷达
    uint8_t  robot_id                = 0;

    /// 机器人等级 (1-3)
    uint8_t  robot_level             = 0;

    /// 机器人当前血量
    uint16_t current_hp              = 0;

    /// 机器人血量上限
    uint16_t maximum_hp              = 0;

    /// 射击热量每秒冷却值
    uint16_t shooter_barrel_cooling_value = 0;

    /// 射击热量上限
    uint16_t shooter_barrel_heat_limit    = 0;

    /// 底盘功率上限, 单位: W
    uint16_t chassis_power_limit     = 0;

    /// 电源管理模块输出情况 (bit 编码):
    /// - bit 0: gimbal 口输出 (0=无输出, 1=24V 输出)
    /// - bit 1: chassis 口输出 (0=无输出, 1=24V 输出)
    /// - bit 2: shooter 口输出 (0=无输出, 1=24V 输出)
    uint8_t  power_management_output = 0;
};

/// @brief 功率热量 (cmd_id: 0x0202, 10Hz, 主控模块→对应机器人)
/// @note V1.2.0: 前 8 字节保留 (uint16_t×2 + float), 不再包含 chassis_power
struct PowerHeat {
    /// 缓冲能量, 单位: J (焦耳)
    uint16_t buffer_energy           = 0;

    /// 17mm 发射机构射击热量
    uint16_t shooter_17mm_barrel_heat = 0;

    /// 42mm 发射机构射击热量
    uint16_t shooter_42mm_barrel_heat = 0;
};

/// @brief 机器人位置 (cmd_id: 0x0203, 1Hz, 主控模块→对应机器人)
/// 坐标系: 场地围挡在红方补给站附近交点为原点, 长边向蓝方为 X+, 短边向红方停机坪为 Y+
struct RobotPos {
    float x     = 0;  ///< 本机位置 x 坐标, 单位: m
    float y     = 0;  ///< 本机位置 y 坐标, 单位: m
    float angle = 0;  ///< 测速模块朝向, 单位: 度 (正北为 0°)
};

/// @brief 机器人增益 (cmd_id: 0x0204, 3Hz, 服务器→对应机器人)
struct Buff {
    /// 回血增益 (百分比, 值 10 表示每秒恢复血量上限的 10%)
    uint8_t  recovery_buff      = 0;

    /// 射击热量冷却增益 (直接值, 值 x 表示热量冷却增加 x/s)
    /// @note V1.2.0: uint16_t, 非 uint8_t
    uint16_t cooling_buff       = 0;

    /// 防御增益 (百分比, 值 50 表示 50% 防御增益)
    uint8_t  defence_buff       = 0;

    /// 负防御增益 (百分比, 值 30 表示 -30% 防御增益, 即受到更多伤害)
    uint8_t  vulnerability_buff = 0;

    /// 攻击增益 (百分比, 值 50 表示 50% 攻击增益)
    uint16_t attack_buff        = 0;

    /// 机器人剩余能量值反馈 (bit 编码)
    /// 仅在剩余能量 <50% 时反馈, 其余默认为 0x80. 初始能量视为 100%
    /// - bit 0: 剩余能量 >= 125%
    /// - bit 1: 剩余能量 >= 100%
    /// - bit 2: 剩余能量 >= 50%
    /// - bit 3: 剩余能量 >= 30%
    /// - bit 4: 剩余能量 >= 15%
    /// - bit 5: 剩余能量 >= 5%
    /// - bit 6: 剩余能量 >= 1%
    uint8_t  remaining_energy   = 0;
};

/// @brief 伤害状态 (cmd_id: 0x0206, 受伤后触发)
/// @note 本地裁判系统判定, 即时发送, 实际是否受伤以服务器最终判定为准
struct HurtData {
    /// 装甲板/测速模块 ID (原始 bit 0-3)
    /// 当扣血原因为弹丸攻击/撞击/离线时, 该值为对应 ID; 其他原因为 0
    uint8_t armor_id  = 0;

    /// 血量变化类型 (原始 bit 4-7):
    /// - 0: 装甲模块被弹丸攻击导致扣血
    /// - 1: 装甲模块或超级电容管理模块离线导致扣血
    /// - 5: 装甲模块受到撞击导致扣血
    uint8_t hurt_type = 0;
};

/// @brief 射击信息 (cmd_id: 0x0207, 弹丸发射后触发)
struct ShootData {
    /// 弹丸类型: 1=17mm, 2=42mm
    uint8_t bullet_type  = 0;

    /// 发射机构 ID: 1=17mm 发射机构, 2=保留, 3=42mm 发射机构
    uint8_t shooter_num  = 0;

    /// 弹丸射频, 单位: Hz
    uint8_t launching_frequency = 0;

    /// 弹丸初速度, 单位: m/s
    float   initial_speed = 0;
};

/// @brief 允许发弹量 (cmd_id: 0x0208, 10Hz, 服务器→己方英雄/步兵/哨兵/空中)
struct ProjectileAllowance {
    /// 机器人自身 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_17mm     = 0;

    /// 42mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm     = 0;

    /// 剩余金币数量
    uint16_t remaining_gold_coin           = 0;

    /// 堡垒增益点提供的储备 17mm 允许发弹量 (与是否实际占领堡垒无关)
    uint16_t projectile_allowance_fortress = 0;
};

/// @brief RFID 状态 (cmd_id: 0x0209, 3Hz, 服务器→己方装有 RFID 模块的机器人)
/// @note 所有 RFID 卡仅在赛内生效, 赛外即使检测到也返回 0
struct RfidStatus {
    /// bit 编码 (1=已检测到该增益点 RFID 卡, 0=未检测到):
    /// - bit 0: 己方基地增益点
    /// - bit 1: 己方中央高地增益点
    /// - bit 2: 对方中央高地增益点
    /// - bit 3: 己方梯形高地增益点
    /// - bit 4: 对方梯形高地增益点
    /// - bit 5-6: 己方飞坡 (前/后)
    /// - bit 7-8: 对方飞坡 (前/后)
    /// - bit 9-10: 己方中央高地 (下/上)
    /// - bit 11-12: 对方中央高地 (下/上)
    /// - bit 13-14: 己方公路 (下/上)
    /// - bit 15-16: 对方公路 (下/上)
    /// - bit 17: 己方堡垒增益点
    /// - bit 18: 己方前哨站增益点
    /// - bit 19: 己方与资源区不重叠的补给区 / RMUL 补给区
    /// - bit 20: 己方与资源区重叠的补给区
    /// - bit 21: 己方装配增益点
    /// - bit 22: 对方装配增益点
    /// - bit 23: 中心增益点 (仅 RMUL)
    /// - bit 24: 对方堡垒增益点
    /// - bit 25: 对方前哨站增益点
    /// - bit 26-31: 己方隧道 (公路下/中/上, 梯形低/中/高)
    uint32_t rfid_status   = 0;

    /// 额外 RFID 点位 (隧道对方侧):
    /// - bit 0-2: 对方隧道 (公路下/中/上)
    /// - bit 3-5: 对方隧道 (梯形低/中/高)
    uint8_t  rfid_status_2 = 0;
};

/// @brief 己方地面机器人位置 (cmd_id: 0x020B, 1Hz, 服务器→己方哨兵)
/// 坐标系: 红方补给站附近交点为原点, 单位: m
struct GroundRobotPosition {
    float hero_x       = 0;  ///< 己方英雄 x
    float hero_y       = 0;  ///< 己方英雄 y
    float engineer_x   = 0;  ///< 己方工程 x
    float engineer_y   = 0;  ///< 己方工程 y
    float standard_3_x = 0;  ///< 己方 3 号步兵 x
    float standard_3_y = 0;  ///< 己方 3 号步兵 y
    float standard_4_x = 0;  ///< 己方 4 号步兵 x
    float standard_4_y = 0;  ///< 己方 4 号步兵 y
    float reserved_x   = 0;  ///< 保留
    float reserved_y   = 0;  ///< 保留
};

/// @brief 哨兵自主决策信息同步 (cmd_id: 0x020D, 1Hz, 服务器→己方哨兵)
struct SentryInfo {
    /// bit 编码:
    /// - bit 0-10: 哨兵成功兑换的允许发弹量 (除远程兑换外, 开局=0)
    /// - bit 11-14: 哨兵成功远程兑换允许发弹量的次数 (开局=0)
    /// - bit 15-18: 哨兵成功远程兑换血量的次数 (开局=0)
    /// - bit 19: 是否可确认免费复活 (1=可以)
    /// - bit 20: 是否可兑换立即复活 (1=可以)
    /// - bit 21-30: 兑换立即复活需花费的金币数
    uint32_t sentry_info   = 0;

    /// bit 编码:
    /// - bit 0: 是否处于脱战状态 (1=脱战)
    /// - bit 1-11: 队伍 17mm 允许发弹量的剩余可兑换数
    /// - bit 12-13: 当前姿态 (1=进攻, 2=防御, 3=移动)
    /// - bit 14: 己方能量机关是否可进入激活状态 (1=可激活)
    uint16_t sentry_info_2 = 0;
};

/// @brief 自定义控制器数据 (cmd_id: 0x0302, 30Hz, 图传链路)
struct CustomController {
    uint8_t data[30] = {};  ///< 30B 自定义数据
    bool    updated  = false;  ///< 是否有新数据到达
};

/// @brief 裁判系统聚合数据 (所有 cmd_id 的消费端快照)
/// 通过 Topic 发布, 由 referee_task 100Hz 周期读取
struct RefereeData {
    GameStatus           game_status;           ///< 0x0001 比赛状态 (1Hz)
    GameResult           game_result;           ///< 0x0002 比赛结果 (结束后触发)
    GameRobotHp          game_robot_hp;         ///< 0x0003 己方机器人血量 (3Hz)
    EventData            event_data;            ///< 0x0101 场地事件 (1Hz)
    RefereeWarning       referee_warning;       ///< 0x0104 裁判警告 (判罚触发)
    DartInfo             dart_info;             ///< 0x0105 飞镖信息 (1Hz)
    RobotStatus          robot_status;          ///< 0x0201 机器人状态 (10Hz)
    PowerHeat            power_heat;            ///< 0x0202 功率热量 (10Hz)
    RobotPos             robot_pos;             ///< 0x0203 机器人位置 (1Hz)
    Buff                 buff;                  ///< 0x0204 增益 (3Hz)
    HurtData             hurt_data;             ///< 0x0206 伤害状态 (受伤触发)
    ShootData            shoot_data;            ///< 0x0207 射击信息 (发射触发)
    ProjectileAllowance  projectile_allowance;  ///< 0x0208 允许发弹量 (10Hz)
    RfidStatus           rfid_status;           ///< 0x0209 RFID 状态 (3Hz)
    GroundRobotPosition  ground_robot_position; ///< 0x020B 全场位置 (1Hz, 哨兵专用)
    SentryInfo           sentry_info;           ///< 0x020D 哨兵信息同步 (1Hz)
    CustomController     custom_controller;     ///< 0x0302 自定义控制器 (30Hz, 图传)
};

static_assert(std::is_trivially_copyable_v<GameStatus>);
static_assert(std::is_trivially_copyable_v<GameResult>);
static_assert(std::is_trivially_copyable_v<GameRobotHp>);
static_assert(std::is_trivially_copyable_v<RobotStatus>);
static_assert(std::is_trivially_copyable_v<PowerHeat>);
static_assert(std::is_trivially_copyable_v<ShootData>);
static_assert(std::is_trivially_copyable_v<ProjectileAllowance>);
static_assert(std::is_trivially_copyable_v<RefereeData>);

} // namespace referee
