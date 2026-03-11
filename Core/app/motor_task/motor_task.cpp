#include "motor_task.hpp"
#include "TaskManager.hpp"
#include "sal_dwt.h"

static ChassisMotors chassis_motors;
static GimbalMotors  gimbal_motors;
static ShootMotors   shoot_motors;

static DwtInstance motor_dwt;

void MotorTaskStart() {
    static TaskManager motor_task({
        .name       = "motor",
        .stack_size = 512,
        .priority   = osPriorityNormal,
        .period_ms  = 1,

        .init_func = []() {
            chassis_motors.Init();
            gimbal_motors.Init();
            shoot_motors.Init();
            motor_dwt.DwtGetDeltaT();
        },

        .task_func = []() {
            // 全部 1kHz
            float dt = motor_dwt.DwtGetDeltaT();
            gimbal_motors.Tick(dt);
            chassis_motors.Tick(dt);
            shoot_motors.Tick(dt);

            // 唯一 CAN 写者
            DjiDriver::FlushAll();
        },
    });
}

ChassisMotors& GetChassisMotors() { return chassis_motors; }
GimbalMotors&  GetGimbalMotors()  { return gimbal_motors; }
ShootMotors&   GetShootMotors()   { return shoot_motors; }
