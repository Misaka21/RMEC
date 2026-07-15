#include <gtest/gtest.h>

#include "shoot_motors.hpp"
#include "robot_def.hpp"
#include "dji_driver.hpp"
#include "fake_sal_test_api.hpp"

#include <cstdint>

namespace {

// 发射电机全部挂在 CAN2 高位组 (0x1FF): 摩擦轮 id5/6 → 帧内偏移 0/2,
// 拨弹盘 id7 → 偏移 4 (dji_driver.cpp TYPE_INFO 分组规则)
constexpr uint8_t FRICTION_L_OFF = 0;
constexpr uint8_t FRICTION_R_OFF = 2;
constexpr uint8_t LOADER_OFF     = 4;

// flow 测试约束 (设计 6.2): 单二进制单次 Init, 用例按声明顺序构成时间线
ShootMotors& Shoot() {
    static ShootMotors s;
    return s;
}

// 读取最近一次 FlushAll 的 CAN2 0x1FF 帧中指定偏移的 int16 输出
int16_t LastShootOutput(uint8_t byte_offset) {
    for (auto it = test::SentCanFrames().rbegin();
         it != test::SentCanFrames().rend(); ++it) {
        if (it->handle == &hcan2 && it->std_id == 0x1FF)
            return static_cast<int16_t>((it->data[byte_offset] << 8) |
                                        it->data[byte_offset + 1]);
    }
    ADD_FAILURE() << "no CAN2 0x1FF frame recorded";
    return 0;
}

void PublishAndTick(const ShootCmdData& cmd) {
    shoot_cmd_topic.Publish(cmd);
    Shoot().Tick(0.001f);
    DjiDriver::FlushAll();
}

ShootCmdData MakeCmd(FrictionMode friction, LoaderMode load, uint8_t rest_heat = 255) {
    ShootCmdData cmd{};
    cmd.friction_mode = friction;
    cmd.load_mode     = load;
    cmd.rest_heat     = rest_heat;
    return cmd;
}

// 注入拨弹盘 (M2006, rx 0x207) 编码器旋转反馈, 步长 < 半圈避免过零误判
void InjectLoaderRotationDeg(float deg) {
    static uint16_t ecd = 0;
    int32_t remaining = static_cast<int32_t>(deg * 8192.0f / 360.0f);
    while (remaining > 0) {
        int32_t step = remaining > 2000 ? 2000 : remaining;
        ecd = static_cast<uint16_t>((ecd + step) % 8192);
        uint8_t frame[8] = {static_cast<uint8_t>(ecd >> 8),
                            static_cast<uint8_t>(ecd & 0xFF),
                            0, 0, 0, 0, 25, 0};
        test::InjectCanRx(&hcan2, 0x200 + LOADER_MOTOR_ID, frame);
        remaining -= step;
    }
}

TEST(ShootLoader, InitAndAllStopSendsZero) {
    Shoot().Init();
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::OFF, LoaderMode::STOP));
    EXPECT_EQ(LastShootOutput(FRICTION_L_OFF), 0);
    EXPECT_EQ(LastShootOutput(FRICTION_R_OFF), 0);
    EXPECT_EQ(LastShootOutput(LOADER_OFF), 0);
}

TEST(ShootLoader, FrictionOnSpinsOpposite) {
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::STOP));
    EXPECT_GT(LastShootOutput(FRICTION_L_OFF), 0);
    EXPECT_LT(LastShootOutput(FRICTION_R_OFF), 0);
    EXPECT_EQ(LastShootOutput(LOADER_OFF), 0);  // STOP 时拨弹盘失能
}

TEST(ShootLoader, ReverseSpinsLoaderNegative) {
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::REVERSE));
    EXPECT_LT(LastShootOutput(LOADER_OFF), 0);
}

TEST(ShootLoader, SingleShotArmsOnEdgeOnly) {
    // REVERSE → STOP 建立边沿基线
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::STOP));

    // STOP → SINGLE 边沿: 目标 = 当前 + 36°×36, 输出正向推进
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::SINGLE));
    EXPECT_GT(LastShootOutput(LOADER_OFF), 0);

    // 电机转过一发还多 (1350° > 1296°), 继续保持 SINGLE:
    // 目标只在边沿设定一次, 不得随电机转动被推高 (回归: 单发变无限连转)
    InjectLoaderRotationDeg(1350.0f);
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::SINGLE));
    EXPECT_LE(LastShootOutput(LOADER_OFF), 0)
        << "SINGLE 持续期间目标被重设, 单发退化为连转";
}

TEST(ShootLoader, HeatZeroStopsLoaderAndDropsQueuedShot) {
    // 剩余热量 0: 拨弹盘立即停转 (SINGLE 仍在保持)
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::SINGLE, 0));
    EXPECT_EQ(LastShootOutput(LOADER_OFF), 0);

    // 热量恢复: 锁定期间滞留的角度目标必须已被丢弃, 不允许补射
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::SINGLE, 255));
    int16_t out = LastShootOutput(LOADER_OFF);
    // 目标已收敛到当前位置: 输出应接近 0 (M2006 raw = 1000/A, 0.5A 裕量)
    EXPECT_LT(out, 500) << "热量恢复瞬间补射了锁定期间排队的弹丸";
    EXPECT_GT(out, -500);
}

TEST(ShootLoader, BurstSpinsLoaderPositive) {
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::BURST));
    EXPECT_GT(LastShootOutput(LOADER_OFF), 0);
}

TEST(ShootLoader, ReverseAllowedEvenAtZeroHeat) {
    // 退弹不受热量保护限制 (rest_heat==0 && mode!=REVERSE 才停转)
    test::ResetFakeRecords();
    PublishAndTick(MakeCmd(FrictionMode::ON, LoaderMode::REVERSE, 0));
    EXPECT_LT(LastShootOutput(LOADER_OFF), 0);
}

}  // namespace
