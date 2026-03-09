#include "TaskManager.hpp"

// Motor 模块 headers 编译验证
#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "mit_passthrough.hpp"

// 类型别名验证: 确认模板实例化正确
using DjiCascadeMotor = Motor<DjiDriver, CascadePid>;
using DjiMitMotor     = Motor<DjiDriver, MitPassthrough>;
