#pragma once

#include "remote.hpp"
#include "dt7_protocol.hpp"

/// 遥控器类型别名
using Dt7Remote = remote::Remote<remote::Dt7Protocol>;

#ifdef __cplusplus
extern "C" {
#endif

/// 初始化遥控器 (创建 Remote + Daemon 实例, 启动 UART DMA 接收)
/// 由 RobotInit() 调用, 必须在调度器启动前完成
void RemoteInit();

#ifdef __cplusplus
}
#endif

/// 遥控器在线判断 (供 cmd_task 等消费者使用)
bool RemoteIsOnline();

/// 获取遥控器实例（必要时用于调试）
Dt7Remote* GetRemote();
