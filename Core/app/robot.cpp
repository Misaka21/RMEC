#include "TaskManager.hpp"

// App 层
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "ins_task.hpp"
#include "remote_task.hpp"
#include "motor_task.hpp"
#include "daemon_task.hpp"
#include "comm_task.hpp"

// Module 层 (编译验证)
#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "mit_passthrough.hpp"
#include "power_limiter.hpp"
#include "topic.hpp"
#include "ahrs_math.hpp"
#include "quaternion_ekf.hpp"
#include "ins_data.hpp"
#include "ins.hpp"
#include "remote.hpp"
#include "dt7_protocol.hpp"

// 编译验证 static_assert
static_assert(std::is_trivially_copyable_v<InsData>);
static_assert(std::is_trivially_copyable_v<remote::Dt7Data>);
static_assert(std::is_trivially_copyable_v<QuaternionEkfOutput>);
static_assert(sizeof(QuaternionEkf) > 0);
static_assert(sizeof(Ins) > 0);
static_assert(sizeof(PowerLimiter) > 0);
static_assert(sizeof(Rls2) > 0);

using DjiCascadeMotor = Motor<DjiDriver, CascadePid>;
using DjiMitMotor     = Motor<DjiDriver, MitPassthrough>;
using Dt7Remote       = remote::Remote<remote::Dt7Protocol>;

// ============================================================
// RobotInit — C++ 侧唯一入口, 由 main.c 在 osKernelStart() 前调用
// ============================================================

extern "C" void RobotInit() {
#if defined(ONE_BOARD)
    // 单板: 全部任务, 无需双板通信
    InsTaskStart();
    RemoteInit();
    MotorTaskStart();
#elif defined(GIMBAL_BOARD)
    // 云台板: IMU + 遥控器 + 云台/发射电机 + 双板通信
    InsTaskStart();
    RemoteInit();
    MotorTaskStart();
    CommTaskStart();
#elif defined(CHASSIS_BOARD)
    // 底盘板: 底盘电机 + 双板通信 (IMU/遥控器数据由云台板转发)
    MotorTaskStart();
    CommTaskStart();
#endif
    DaemonTaskStart();
    // 未来: RobotTaskStart(), UiTaskStart() ...
}
