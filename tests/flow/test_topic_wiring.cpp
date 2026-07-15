#include <gtest/gtest.h>

#include "TaskManager.hpp"
#include "can.h"
#include "motor_task.hpp"
#include "robot_topics.hpp"
#include "sal_dwt.h"
#include "fake_sal_test_api.hpp"

#include <cstdint>

namespace {

// 断言意图 (设计 6.2): 每个 cmd topic 发布后步进消费任务, fake CAN 必须收到
// 非零输出 —— 注释掉 robot_task/motor_task 里的任一发布/消费即变红 (拦截断头链路)

// flow 测试约束: 单二进制单次 init, 用例按声明顺序执行

// 读取最近一帧 (handle, std_id) 中指定偏移的 int16 输出
int16_t LastOutput(CAN_HandleTypeDef* handle, uint32_t std_id, uint8_t byte_offset) {
    for (auto it = test::SentCanFrames().rbegin();
         it != test::SentCanFrames().rend(); ++it) {
        if (it->handle == handle && it->std_id == std_id)
            return static_cast<int16_t>((it->data[byte_offset] << 8) |
                                        it->data[byte_offset + 1]);
    }
    ADD_FAILURE() << "no frame handle=" << handle->id << " std_id=0x" << std::hex
                  << std_id;
    return 0;
}

void StepMotorTask() {
    test::AdvanceTimeMs(1);  // motor_dwt.DwtGetDeltaT() → 1 ms
    test::StepTask("motor");
}

TEST(TopicWiring, MotorTaskRegistersAndInits) {
    DwtInstance::DwtInit(168);
    MotorTaskStart();  // ONE_BOARD: chassis + gimbal + shoot 全部创建
    bool has_motor = false;
    for (auto& t : test::TaskRegistry()) has_motor |= (t.name == "motor");
    ASSERT_TRUE(has_motor);
    test::RunTaskInit("motor");
}

TEST(TopicWiring, GimbalCmdDrivesCan1VoltageGroup) {
    test::ResetFakeRecords();
    GimbalCmdData cmd{};
    cmd.mode  = GimbalMode::GYRO_MODE;
    cmd.yaw   = 10.0f;   // IMU 反馈为零 → 角度误差 → 非零电压
    cmd.pitch = 5.0f;
    gimbal_cmd_topic.Publish(cmd);
    StepMotorTask();
    // GM6020 电压组: CAN1 0x1FF, yaw id1 → 偏移 0, pitch id2 → 偏移 2
    EXPECT_NE(LastOutput(&hcan1, 0x1FF, 0), 0);
    EXPECT_NE(LastOutput(&hcan1, 0x1FF, 2), 0);
}

TEST(TopicWiring, GimbalZeroForceSendsZero) {
    test::ResetFakeRecords();
    GimbalCmdData cmd{};
    cmd.mode = GimbalMode::ZERO_FORCE;
    gimbal_cmd_topic.Publish(cmd);
    StepMotorTask();
    EXPECT_EQ(LastOutput(&hcan1, 0x1FF, 0), 0);
    EXPECT_EQ(LastOutput(&hcan1, 0x1FF, 2), 0);
}

TEST(TopicWiring, ChassisCmdDrivesCan2LowGroup) {
    test::ResetFakeRecords();
    ChassisCmdData cmd{};
    cmd.mode = ChassisMode::NO_FOLLOW;
    cmd.vx   = 5000.0f;  // deg/s, 四轮同向
    chassis_cmd_topic.Publish(cmd);
    StepMotorTask();
    // 底盘 M3508 id1-4: CAN2 0x200, 偏移 0/2/4/6
    for (uint8_t off : {0, 2, 4, 6})
        EXPECT_NE(LastOutput(&hcan2, 0x200, off), 0) << "wheel offset " << int(off);
}

TEST(TopicWiring, ShootCmdDrivesCan2HighGroup) {
    test::ResetFakeRecords();
    ShootCmdData cmd{};
    cmd.friction_mode = FrictionMode::ON;
    cmd.load_mode     = LoaderMode::BURST;
    cmd.rest_heat     = 255;
    shoot_cmd_topic.Publish(cmd);
    StepMotorTask();
    // 摩擦轮 id5/6 → CAN2 0x1FF 偏移 0/2 (对转), 拨弹盘 id7 → 偏移 4
    EXPECT_GT(LastOutput(&hcan2, 0x1FF, 0), 0);
    EXPECT_LT(LastOutput(&hcan2, 0x1FF, 2), 0);
    EXPECT_GT(LastOutput(&hcan2, 0x1FF, 4), 0);
}

}  // namespace
