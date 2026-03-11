# daemon_task

Daemon 看门狗 tick 任务, 100 Hz FreeRTOS 任务。

## 工作流程

每 10ms 调用 `daemon::DaemonInstance::TickAll()`, 遍历所有已注册的 `DaemonInstance`, 驱动 FSM 状态迁移并触发离线/恢复回调。

## 代码结构

| 文件 | 说明 |
|---|---|
| `daemon_task.hpp` | `DaemonTaskStart()` 声明 |
| `daemon_task.cpp` | TaskManager 包装, 128 字栈 |

## 条件编译

所有板型 (`ONE_BOARD` / `GIMBAL_BOARD` / `CHASSIS_BOARD`) 均启动此任务。

## 参数计算

`DaemonInstance` 的 `timeout_ticks` 基于此任务的 100 Hz 频率:

| timeout_ticks | 实际超时 |
|---|---|
| 10 | 100 ms |
| 50 | 500 ms |
| 100 | 1 s |

详见 [daemon 模块文档](../../module/daemon/README.md)。
