#include <gtest/gtest.h>

#include "TaskManager.hpp"
#include "robot_task.hpp"
#include "robot_topics.hpp"
#include "math.hpp"

#include "remote_task.hpp"
#include "vision_task.hpp"
#include "ins_task.hpp"
#include "referee_task.hpp"

#include <cstdint>

// ======================== 联机状态桩 ========================
// robot_task.cpp 引用的跨 TU helper 在此用可控开关替换 (链接期替换真 task 实现)

static bool g_remote_online  = false;
static bool g_ins_ready      = true;
static bool g_vision_online  = false;
static bool g_referee_online = false;
static vision::AimMode g_last_aim = vision::AimMode::OFF;

bool RemoteIsOnline() { return g_remote_online; }
bool InsIsReady() { return g_ins_ready; }
bool VisionIsOnline() { return g_vision_online; }
bool RefereeIsOnline() { return g_referee_online; }
void VisionSetMode(vision::AimMode mode, vision::EnemyColor, float) {
    g_last_aim = mode;
}

namespace {

using remote::SwitchPos;

// flow 测试约束 (设计 6.2): 单二进制单次 init, 用例按声明顺序构成时间线

TopicReader<GimbalCmdData>*  gimbal_out  = nullptr;
TopicReader<ChassisCmdData>* chassis_out = nullptr;
TopicReader<ShootCmdData>*   shoot_out   = nullptr;

remote::Dt7Data MakeRc(SwitchPos sw_r, SwitchPos sw_l = SwitchPos::MID) {
    remote::Dt7Data rc{};
    rc.sw_r = sw_r;
    rc.sw_l = sw_l;
    return rc;
}

void PublishRcAndStep(const remote::Dt7Data& rc) {
    remote_topic.Publish(rc);
    test::StepTask("robot");
}

// Latest(): 非消费式读取, 同一周期可多次断言
GimbalCmdData  Gimbal()   { return gimbal_out->Latest(); }
ChassisCmdData Chassis()  { return chassis_out->Latest(); }
ShootCmdData   ShootCmd() { return shoot_out->Latest(); }

TEST(RobotTaskModes, StartRegistersAndInits) {
    gimbal_out  = gimbal_cmd_topic.Subscribe();
    chassis_out = chassis_cmd_topic.Subscribe();
    shoot_out   = shoot_cmd_topic.Subscribe();
    ASSERT_NE(gimbal_out, nullptr);

    RobotTaskStart();
    ASSERT_EQ(test::TaskRegistry().size(), 1u);
    EXPECT_EQ(test::TaskRegistry()[0].name, "robot");
    test::RunTaskInit("robot");
}

TEST(RobotTaskModes, RemoteOfflineForcesZeroForce) {
    g_remote_online = false;
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    EXPECT_EQ(Gimbal().mode, GimbalMode::ZERO_FORCE);
    EXPECT_EQ(Chassis().mode, ChassisMode::ZERO_FORCE);
    // 裁判系统从未上线: 视为台架调试, 不限热量
    EXPECT_EQ(ShootCmd().rest_heat, 255);
    EXPECT_EQ(ShootCmd().friction_mode, FrictionMode::OFF);
}

TEST(RobotTaskModes, SwitchDownIsZeroForce) {
    g_remote_online = true;
    PublishRcAndStep(MakeRc(SwitchPos::DOWN));
    EXPECT_EQ(Gimbal().mode, GimbalMode::ZERO_FORCE);
    EXPECT_EQ(Chassis().mode, ChassisMode::ZERO_FORCE);
}

TEST(RobotTaskModes, SwitchMidIsGyroNoFollow) {
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    EXPECT_EQ(Gimbal().mode, GimbalMode::GYRO_MODE);
    EXPECT_EQ(Chassis().mode, ChassisMode::NO_FOLLOW);
    EXPECT_EQ(g_last_aim, vision::AimMode::OFF);
}

TEST(RobotTaskModes, InsNotReadyBlocksGyroMode) {
    // IMU 初始化失败时禁止云台上力, 底盘不受影响
    g_ins_ready = false;
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    EXPECT_EQ(Gimbal().mode, GimbalMode::ZERO_FORCE);
    EXPECT_EQ(Chassis().mode, ChassisMode::NO_FOLLOW);
    g_ins_ready = true;
}

TEST(RobotTaskModes, SlowRemoteDoesNotFlapMode) {
    // 遥控 ~70Hz < 任务 200Hz: 只发一帧, 连续步进 4 次, 模式必须保持
    // (回归: 无新数据时若用局部零值会抖回 ZERO_FORCE)
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    test::StepTask("robot", 4);
    EXPECT_EQ(Gimbal().mode, GimbalMode::GYRO_MODE);
    EXPECT_EQ(Chassis().mode, ChassisMode::NO_FOLLOW);
}

TEST(RobotTaskModes, InvalidSwitchValueIsZeroForce) {
    // 非法拨杆原始值 0 (协议层透传) 落入 default 分支
    PublishRcAndStep(MakeRc(static_cast<SwitchPos>(0)));
    EXPECT_EQ(Gimbal().mode, GimbalMode::ZERO_FORCE);
    EXPECT_EQ(Chassis().mode, ChassisMode::ZERO_FORCE);
}

TEST(RobotTaskModes, VisionTakeoverSetsAbsoluteTargets) {
    g_vision_online = true;
    vision::VisionRxData v{};
    v.control = 1;
    v.yaw   = 0.5f;    // rad
    v.pitch = -0.2f;   // rad
    vision_topic.Publish(v);
    PublishRcAndStep(MakeRc(SwitchPos::UP));
    EXPECT_EQ(Gimbal().mode, GimbalMode::GYRO_MODE);
    EXPECT_NEAR(Gimbal().yaw, 0.5f * math::RAD_2_DEGREE, 1e-3f);
    EXPECT_NEAR(Gimbal().pitch, -0.2f * math::RAD_2_DEGREE, 1e-3f);
    EXPECT_EQ(g_last_aim, vision::AimMode::AUTO_AIM);
}

TEST(RobotTaskModes, StickIntegratesTargetWhenNoVision) {
    g_vision_online = false;
    float yaw_before = Gimbal().yaw;
    auto rc = MakeRc(SwitchPos::MID);
    rc.ch_r_x = 660;  // 满杆: +800 deg/s × 5 ms = +4°/步
    PublishRcAndStep(rc);
    EXPECT_NEAR(Gimbal().yaw, yaw_before + 4.0f, 1e-3f);
    EXPECT_EQ(g_last_aim, vision::AimMode::OFF);
}

TEST(RobotTaskModes, DialControlsLoader) {
    auto rc = MakeRc(SwitchPos::MID, SwitchPos::UP);

    rc.dial = -660;  // 下压连发
    PublishRcAndStep(rc);
    EXPECT_EQ(ShootCmd().friction_mode, FrictionMode::ON);
    EXPECT_EQ(ShootCmd().load_mode, LoaderMode::BURST);

    rc.dial = 660;   // 上抬退弹
    PublishRcAndStep(rc);
    EXPECT_EQ(ShootCmd().load_mode, LoaderMode::REVERSE);

    rc.dial = 0;     // 回中停止
    PublishRcAndStep(rc);
    EXPECT_EQ(ShootCmd().load_mode, LoaderMode::STOP);

    rc.sw_l = SwitchPos::MID;  // 摩擦轮关: 全部回默认
    PublishRcAndStep(rc);
    EXPECT_EQ(ShootCmd().friction_mode, FrictionMode::OFF);
    EXPECT_EQ(ShootCmd().load_mode, LoaderMode::STOP);
}

TEST(RobotTaskModes, HeatLatchSurvivesRefereeDropout) {
    // 裁判在线: rest = heat_limit - heat = 100 - 30 = 70
    g_referee_online = true;
    referee::RefereeData ref{};
    ref.robot_status.shooter_barrel_heat_limit = 100;
    ref.power_heat.shooter_17mm_barrel_heat    = 30;
    referee_topic.Publish(ref);
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    EXPECT_EQ(ShootCmd().rest_heat, 70);

    // 比赛中链路闪断: 锁存最后一次真实值, 不得回跳 255 绕过热量保护
    g_referee_online = false;
    PublishRcAndStep(MakeRc(SwitchPos::MID));
    EXPECT_EQ(ShootCmd().rest_heat, 70);
}

}  // namespace
