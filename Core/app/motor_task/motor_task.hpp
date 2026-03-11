#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// 启动电机控制任务 (1 kHz, osPriorityNormal)
/// 由 RobotInit() 调用
void MotorTaskStart();

#ifdef __cplusplus
}
#endif
