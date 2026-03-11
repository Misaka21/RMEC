#pragma once

#include "chassis_motors.hpp"
#include "gimbal_motors.hpp"
#include "shoot_motors.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void MotorTaskStart();

#ifdef __cplusplus
}
#endif

ChassisMotors& GetChassisMotors();
GimbalMotors&  GetGimbalMotors();
ShootMotors&   GetShootMotors();
