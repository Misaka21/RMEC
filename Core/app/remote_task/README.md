# remote_task

遥控器 App 层编排: 初始化 + ISR 回调发布。

> Remote 模块本身不是 FreeRTOS task, 它是 UART 中断回调驱动的数据源。
> 本文件负责创建实例并注入发布回调, 将模块与 Topic 系统连接。

## 工作流程

### 初始化 (RemoteInit)

1. 从 `robot_def.hpp` 获取 UART 句柄 (`RC_UART_HANDLE`)
2. 创建 `Remote<Dt7Protocol>` 实例, 注入 publish 回调
3. 回调 lambda 将解析后的 `Dt7Data` 发布到 `remote_topic`
4. 构造函数内启动 UART DMA 接收

### 数据发布 (ISR 驱动)

```
UART DMA Complete ISR
  → HAL_UARTEx_RxEventCallback
    → sal::UartInstance rx_cbk_
      → Remote::OnReceive()
        → Dt7Protocol::Decode()    // 解析 18 字节 DBUS 帧
        → publish_(data_)          // 注入的 lambda → remote_topic.Publish()
```

发布频率由遥控器决定 (~70 Hz), 不依赖任何定时器或轮询。

### 离线检测

`Remote::Tick()` 由消费者任务 (cmd_task) 定期调用:
- 每次调用递增离线计数
- 超时后清零数据并发布 (安全状态), 重启 UART DMA

## 外部接口

```cpp
void RemoteInit();              // 由 RobotInit() 调用
Dt7Remote* GetRemote();         // 获取实例, 用于 IsOnline() / Tick()
```
