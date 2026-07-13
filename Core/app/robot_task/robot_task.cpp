#include "robot_task.hpp"
#include "TaskManager.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "math.hpp"
#include "vision_task.hpp"
#include "remote_task.hpp"
#include "referee_task.hpp"
#include "ins_task.hpp"

// ======================== 灵敏度常量 ========================

// 摇杆满杆对应的角速度 (deg/s)
static constexpr float YAW_SENSITIVITY   = 800.0f;
static constexpr float PITCH_SENSITIVITY = 1000.0f;

// 摇杆满杆对应的底盘轮速 (电机转子 deg/s) // TODO: 实测标定
static constexpr float CHASSIS_SPEED_SENSITIVITY = 20000.0f;

// 拨轮触发阈值 (满量程 660 的一半)
static constexpr int16_t DIAL_THRESHOLD = 330;

// 遥控器通道满杆值
static constexpr float CH_FULL_SCALE = 660.0f;

// 任务周期
static constexpr uint32_t ROBOT_PERIOD_MS = 5;  // 200 Hz
static constexpr float    ROBOT_DT        = static_cast<float>(ROBOT_PERIOD_MS) / 1000.0f;

// ======================== 静态状态 ========================

static TopicReader<remote::Dt7Data>*       rc_reader          = nullptr;
static TopicReader<vision::VisionRxData>*  vision_reader      = nullptr;
static TopicReader<GimbalFeedData>*        gimbal_feed_reader = nullptr;
static TopicReader<referee::RefereeData>*  referee_reader     = nullptr;

static float yaw_target   = 0;
static float pitch_target = 0;
static GimbalMode last_mode = GimbalMode::ZERO_FORCE;

// ======================== 实现 ========================

static void RobotInit() {
    rc_reader          = remote_topic.Subscribe();
    vision_reader      = vision_topic.Subscribe();
    gimbal_feed_reader = gimbal_feed_topic.Subscribe();
    referee_reader     = referee_topic.Subscribe();
}

static void RobotTick() {
    // 读取遥控器数据: 无新数据时保持上次有效帧 (任务 200Hz > 遥控 ~70Hz,
    // 若用局部零值会导致模式每周期抖回 ZERO_FORCE)
    static remote::Dt7Data rc{};
    rc_reader->Read(rc);

    // 读取视觉数据 (同理保持上次值)
    static vision::VisionRxData vision{};
    vision_reader->Read(vision);

    // 读取云台反馈
    static GimbalFeedData feed{};
    gimbal_feed_reader->Read(feed);

    // 读取裁判系统数据 (无发布者的板型上保持默认值)
    static referee::RefereeData ref{};
    referee_reader->Read(ref);

    // ---- 模式判断 ----
    // 遥控器离线时无条件 ZERO_FORCE; 非法拨杆值 (如 0) 落入 default 同样 ZERO_FORCE
    GimbalMode  mode         = GimbalMode::ZERO_FORCE;
    ChassisMode chassis_mode = ChassisMode::ZERO_FORCE;
    const bool  rc_online    = RemoteIsOnline();

    if (rc_online) {
        switch (rc.sw_r) {
        case remote::SwitchPos::DOWN:
            mode         = GimbalMode::ZERO_FORCE;
            chassis_mode = ChassisMode::ZERO_FORCE;
            break;
        case remote::SwitchPos::MID:
        case remote::SwitchPos::UP:
            // IMU 初始化失败时禁止云台上力: 冻结的姿态反馈会让 PID 误差
            // 无界增长, 云台全力矩撞限位
            mode         = InsIsReady() ? GimbalMode::GYRO_MODE
                                        : GimbalMode::ZERO_FORCE;
            chassis_mode = ChassisMode::NO_FOLLOW;
            break;
        default:
            break;
        }
    }

    // 模式切换同步: ZERO_FORCE → GYRO_MODE 时从反馈同步目标角度
    if (mode == GimbalMode::GYRO_MODE && last_mode == GimbalMode::ZERO_FORCE) {
        yaw_target   = feed.yaw_total;
        pitch_target = feed.pitch;
    }

    if (mode == GimbalMode::GYRO_MODE) {
        bool vision_active = (rc.sw_r == remote::SwitchPos::UP)
                             && VisionIsOnline()
                             && (vision.control == 1);

        if (vision_active) {
            // 视觉接管: 绝对目标角度 (视觉发弧度, PID 用度)
            yaw_target   = vision.yaw * math::RAD_2_DEGREE;
            pitch_target = vision.pitch * math::RAD_2_DEGREE;
        } else {
            // 遥控器控制: 增量式
            float ch_yaw   = static_cast<float>(rc.ch_r_x) / CH_FULL_SCALE;
            float ch_pitch = static_cast<float>(rc.ch_r_y) / CH_FULL_SCALE;

            yaw_target   += ch_yaw   * YAW_SENSITIVITY   * ROBOT_DT;
            pitch_target += ch_pitch * PITCH_SENSITIVITY  * ROBOT_DT;
        }

        pitch_target = math::Clamp(pitch_target, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
    }

    last_mode = mode;

    // 根据右拨杆实时更新视觉自瞄模式
    vision::AimMode aim = (rc_online && rc.sw_r == remote::SwitchPos::UP)
                          ? vision::AimMode::AUTO_AIM
                          : vision::AimMode::OFF;
    VisionSetMode(aim, vision::EnemyColor::UNKNOWN, 15.0f);

    // ---- 发布云台命令 ----
    GimbalCmdData cmd{};
    cmd.yaw   = yaw_target;
    cmd.pitch = pitch_target;
    cmd.mode  = mode;
    gimbal_cmd_topic.Publish(cmd);

    // ---- 发布底盘命令 ----
    ChassisCmdData ccmd{};
    ccmd.mode = chassis_mode;
    if (chassis_mode != ChassisMode::ZERO_FORCE) {
        ccmd.vx = static_cast<float>(rc.ch_l_y) / CH_FULL_SCALE * CHASSIS_SPEED_SENSITIVITY;
        ccmd.vy = static_cast<float>(rc.ch_l_x) / CH_FULL_SCALE * CHASSIS_SPEED_SENSITIVITY;
        ccmd.wz = 0;  // 旋转通道暂未分配
    }
    if (RefereeIsOnline()) {
        ccmd.power_limit   = static_cast<float>(ref.robot_status.chassis_power_limit);
        ccmd.buffer_energy = static_cast<float>(ref.power_heat.buffer_energy);
    }
    chassis_cmd_topic.Publish(ccmd);

    // ---- 发布发射命令 ----
    ShootCmdData scmd{};
    if (rc_online && rc.sw_l == remote::SwitchPos::UP) {
        scmd.friction_mode = FrictionMode::ON;
        // 拨轮下压连发, 上抬退弹 (方向若与实车相反, 调换两处阈值符号)
        if (rc.dial < -DIAL_THRESHOLD)
            scmd.load_mode = LoaderMode::BURST;
        else if (rc.dial > DIAL_THRESHOLD)
            scmd.load_mode = LoaderMode::REVERSE;
        else
            scmd.load_mode = LoaderMode::STOP;
    }
    // 剩余热量: 裁判在线用真实值; 从未上线视为台架调试不限制;
    // 比赛中链路闪断则锁存最后一次真实值, 避免掉线瞬间绕过热量保护
    static uint8_t rest_heat_latch = 255;
    if (RefereeIsOnline()) {
        int32_t rest = static_cast<int32_t>(ref.robot_status.shooter_barrel_heat_limit)
                     - static_cast<int32_t>(ref.power_heat.shooter_17mm_barrel_heat);
        rest_heat_latch = static_cast<uint8_t>(math::Clamp(static_cast<float>(rest), 0.0f, 255.0f));
    }
    scmd.rest_heat = rest_heat_latch;
    shoot_cmd_topic.Publish(scmd);
}

void RobotTaskStart() {
    static TaskManager robot_task({
        .name       = "robot",
        .stack_size = 128,
        .priority   = osPriorityAboveNormal,
        .period_ms  = ROBOT_PERIOD_MS,
        .init_func  = RobotInit,
        .task_func  = RobotTick,
    });
}
