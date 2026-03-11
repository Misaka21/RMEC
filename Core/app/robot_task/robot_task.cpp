#include "robot_task.hpp"
#include "TaskManager.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "general_def.hpp"
#include "vision_task.hpp"

// ======================== 灵敏度常量 ========================

// 摇杆满杆对应的角速度 (deg/s)
static constexpr float YAW_SENSITIVITY   = 300.0f;
static constexpr float PITCH_SENSITIVITY = 200.0f;

// 遥控器通道满杆值
static constexpr float CH_FULL_SCALE = 660.0f;

// 任务周期
static constexpr uint32_t ROBOT_PERIOD_MS = 5;  // 200 Hz
static constexpr float    ROBOT_DT        = static_cast<float>(ROBOT_PERIOD_MS) / 1000.0f;

// ======================== 静态状态 ========================

static TopicReader<remote::Dt7Data>*       rc_reader          = nullptr;
static TopicReader<vision::VisionRxData>*  vision_reader      = nullptr;
static TopicReader<GimbalFeedData>*        gimbal_feed_reader = nullptr;

static float yaw_target   = 0;
static float pitch_target = 0;
static GimbalMode last_mode = GimbalMode::ZERO_FORCE;

// ======================== 实现 ========================

static void RobotInit() {
    rc_reader          = remote_topic.Subscribe();
    vision_reader      = vision_topic.Subscribe();
    gimbal_feed_reader = gimbal_feed_topic.Subscribe();
}

static void RobotTick() {
    // 读取遥控器数据
    remote::Dt7Data rc{};
    rc_reader->Read(rc);

    // 读取视觉数据
    vision::VisionRxData vision{};
    vision_reader->Read(vision);

    // 读取云台反馈
    GimbalFeedData feed{};
    gimbal_feed_reader->Read(feed);

    // 模式判断
    GimbalMode mode = GimbalMode::ZERO_FORCE;

    switch (rc.sw_r) {
    case remote::SwitchPos::DOWN:
        mode = GimbalMode::ZERO_FORCE;
        break;
    case remote::SwitchPos::MID:
        mode = GimbalMode::GYRO_MODE;
        break;
    case remote::SwitchPos::UP:
        mode = GimbalMode::GYRO_MODE;
        break;
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
            yaw_target   = vision.yaw * RAD_2_DEGREE;
            pitch_target = vision.pitch * RAD_2_DEGREE;
        } else {
            // 遥控器控制: 增量式
            float ch_yaw   = static_cast<float>(rc.ch_r_x) / CH_FULL_SCALE;
            float ch_pitch = static_cast<float>(rc.ch_r_y) / CH_FULL_SCALE;

            yaw_target   += ch_yaw   * YAW_SENSITIVITY   * ROBOT_DT;
            pitch_target += ch_pitch * PITCH_SENSITIVITY  * ROBOT_DT;
        }

        pitch_target = Clamp(pitch_target, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
    }

    last_mode = mode;

    // 根据右拨杆实时更新视觉自瞄模式
    vision::AimMode aim = (rc.sw_r == remote::SwitchPos::UP)
                          ? vision::AimMode::AUTO_AIM
                          : vision::AimMode::OFF;
    VisionSetMode(aim, vision::EnemyColor::UNKNOWN, 15.0f);

    // 发布云台命令
    GimbalCmdData cmd{};
    cmd.yaw   = yaw_target;
    cmd.pitch = pitch_target;
    cmd.mode  = mode;
    gimbal_cmd_topic.Publish(cmd);
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
