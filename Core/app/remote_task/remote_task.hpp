#pragma once

#include "remote.hpp"
#include "dt7_protocol.hpp"

/// 遥控器类型别名
using Dt7Remote = remote::Remote<remote::Dt7Protocol>;

#ifdef __cplusplus
extern "C" {
#endif

/// 初始化遥控器 (创建 Remote 实例, 启动 UART DMA 接收)
/// 由 RobotInit() 调用, 必须在调度器启动前完成
void RemoteInit();

/// 离线检测: 比较快照序号, 超时则发布零帧并重启接收
/// 建议由 cmd_task 周期调用（不新增独立任务）
void RemoteOfflineCheck();

#ifdef __cplusplus
}
#endif

/// 获取遥控器实例（必要时用于调试）
Dt7Remote* GetRemote();
