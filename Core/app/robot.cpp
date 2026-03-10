#include "TaskManager.hpp"

// App 层
#include "robot_def.hpp"
#include "robot_types.hpp"
#include "robot_topics.hpp"
#include "ins_task.hpp"

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

// 编译验证 static_assert
static_assert(std::is_trivially_copyable_v<InsData>);
static_assert(std::is_trivially_copyable_v<QuaternionEkfOutput>);
static_assert(sizeof(QuaternionEkf) > 0);
static_assert(sizeof(Ins) > 0);
static_assert(sizeof(PowerLimiter) > 0);
static_assert(sizeof(Rls2) > 0);

using DjiCascadeMotor = Motor<DjiDriver, CascadePid>;
using DjiMitMotor     = Motor<DjiDriver, MitPassthrough>;

// ============================================================
// RobotInit — C++ 侧唯一入口, 由 main.c 在 osKernelStart() 前调用
// ============================================================

extern "C" void RobotInit() {
    InsTaskStart();
    // 未来: RobotTaskStart(), DaemonTaskStart(), UiTaskStart() ...
}
