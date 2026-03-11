#include "motor_task.hpp"
#include "chassis_motors.hpp"
#include "gimbal_motors.hpp"
#include "shoot_motors.hpp"
#include "TaskManager.hpp"
#include "sal_dwt.h"

// 新增 driver 头文件 (FlushAll 调用)
#include "dji_driver.hpp"
#include "dm_driver.hpp"
#include "ht_driver.hpp"
#include "lk_driver.hpp"

static ChassisMotors chassis;
static GimbalMotors  gimbal;
static ShootMotors   shoot;

static DwtInstance motor_dwt;

void MotorTaskStart() {
    static TaskManager motor_task({
        .name       = "motor",
        .stack_size = 512,
        .priority   = osPriorityNormal,
        .period_ms  = 1,

        .init_func = []() {
            chassis.Init();
            gimbal.Init();
            shoot.Init();
            motor_dwt.DwtGetDeltaT();
        },

        .task_func = []() {
            float dt = motor_dwt.DwtGetDeltaT();

            // 全部 1kHz 更新
            gimbal.Tick(dt);
            chassis.Tick(dt);
            shoot.Tick(dt);

            // 统一 flush 所有驱动类型 (无实例时立即返回)
            DjiDriver::FlushAll();
            DmDriver::FlushAll();
            HtDriver::FlushAll();
            LkDriver::FlushAll();
        },
    });
}
