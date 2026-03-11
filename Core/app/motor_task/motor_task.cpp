#include "motor_task.hpp"
#include "robot_def.hpp"
#include "TaskManager.hpp"
#include "sal_dwt.h"

// 按板型条件编译子系统
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis_motors.hpp"
static ChassisMotors chassis;
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal_motors.hpp"
#include "shoot_motors.hpp"
static GimbalMotors gimbal;
static ShootMotors  shoot;
#endif

// driver 头文件 (FlushAll 调用)
#include "dji_driver.hpp"
#include "dm_driver.hpp"
#include "ht_driver.hpp"
#include "lk_driver.hpp"

static DwtInstance motor_dwt;

void MotorTaskStart() {
    // Pre-scheduler: Subscribe + 创建电机 (满足 Topic 时序约束)
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    chassis.Init();
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    gimbal.Init();
    shoot.Init();
#endif

    static TaskManager motor_task({
        .name       = "motor",
        .stack_size = 512,
        .priority   = osPriorityNormal,
        .period_ms  = 1,

        .init_func = []() {
            motor_dwt.DwtGetDeltaT();
        },

        .task_func = []() {
            float dt = motor_dwt.DwtGetDeltaT();

            // 全部 1kHz 更新
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
            gimbal.Tick(dt);
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            chassis.Tick(dt);
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
            shoot.Tick(dt);
#endif

            // 统一 flush 所有驱动类型 (无实例时立即返回)
            DjiDriver::FlushAll();
            DmDriver::FlushAll();
            HtDriver::FlushAll();
            LkDriver::FlushAll();
        },
    });
}
