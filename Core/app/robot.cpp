#include "TaskManager.hpp"

// Motor 模块 headers 编译验证
#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "mit_passthrough.hpp"
#include "power_limiter.hpp"

// Topic 模块编译验证
#include "topic.hpp"

// INS 模块编译验证
#include "ahrs_math.hpp"
#include "quaternion_ekf.hpp"
#include "ins_data.hpp"
#include "ins.hpp"
#include "ins_task.hpp"

struct TopicTestData { float x; float y; };
static_assert(std::is_trivially_copyable_v<TopicTestData>);
static Topic<TopicTestData> topic_test_;

// 类型别名验证: 确认模板实例化正确
using DjiCascadeMotor = Motor<DjiDriver, CascadePid>;
using DjiMitMotor     = Motor<DjiDriver, MitPassthrough>;

// PowerLimiter 编译验证: 确认类型实例化正确
static_assert(sizeof(PowerLimiter) > 0, "PowerLimiter must be instantiable");
static_assert(sizeof(Rls2) > 0, "Rls2 must be instantiable");

// INS 编译验证
static_assert(std::is_trivially_copyable_v<InsData>);
static_assert(std::is_trivially_copyable_v<QuaternionEkfOutput>);
static_assert(sizeof(QuaternionEkf) > 0);
static_assert(sizeof(Ins) > 0);

// 典型用法（Chassis 控制循环内）:
//   PowerLimiter limiter;
//   limiter.Init(cfg);
//   ...
//   PowerMotorState states[4];
//   for (int i = 0; i < 4; ++i) {
//       states[i].pid_output   = motors[i].ComputeOutput(dt);
//       states[i].speed_rad    = motors[i].Measure().speed_aps * DEGREE_2_RAD;
//       states[i].set_speed_rad = motors[i].GetRef() * DEGREE_2_RAD;
//       states[i].max_output   = 16384.0f;
//   }
//   limiter.UpdateEnergyLoop(ref_limit, buf_energy, measured_power, dt);
//   limiter.Limit(states, 4);
//   for (int i = 0; i < 4; ++i)
//       motors[i].ApplyOutput(states[i].pid_output);
//   DjiDriver::FlushAll();
