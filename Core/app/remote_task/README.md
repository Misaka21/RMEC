# remote_task

遥控器 App 层：初始化 + ISR 回调发布 + 离线检测。

> `Remote` 模块在 UART ISR 中解码并通过回调直接发布到 `remote_topic`，零轮询延迟。
> 本目录负责创建模块实例、注入发布回调，并处理离线恢复策略。

## 工作流程

### 初始化 (RemoteInit)

1. 从 `robot_def.hpp` 获取 UART 句柄 (`RC_UART_HANDLE`)
2. 创建 `Remote<Dt7Protocol>` 实例，注入发布回调
3. 启动 UART DMA+IDLE 接收

### 数据路径（ISR 回调直接发布）

```
UART DMA Complete ISR
  → HAL_UARTEx_RxEventCallback
    → sal::UartInstance rx_cbk_
      → Remote::OnReceive()
        → Dt7Protocol::Decode()
        → seqlock 写快照
        → on_publish_(curr)          // 回调发布到 remote_topic
```

`remote_topic` 只有 ISR 回调一个写者，满足 Topic 单生产者约束。

### 离线检测

`RemoteOfflineCheck()` 由 cmd_task 等周期任务调用，比较快照序号：
- 序号有变化：重置离线计数
- 超时未变化：发布一帧零值安全数据，并重启 UART 接收

## 外部接口

```cpp
void RemoteInit();          // 由 RobotInit() 调用
void RemoteOfflineCheck();  // 建议由 cmd_task 周期调用
Dt7Remote* GetRemote();     // 调试用途
```
