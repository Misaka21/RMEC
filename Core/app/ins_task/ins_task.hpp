#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void InsTaskStart();

#ifdef __cplusplus
}
#endif

/// IMU 初始化是否成功 (供 robot_task 门控 GYRO_MODE, 失败时禁止云台上力)
bool InsIsReady();
