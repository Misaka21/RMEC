#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// 启动 Daemon 看门狗任务 (100 Hz, osPriorityNormal)
/// 由 RobotInit() 调用
void DaemonTaskStart();

#ifdef __cplusplus
}
#endif
