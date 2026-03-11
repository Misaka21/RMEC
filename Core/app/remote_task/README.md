# remote_task

遥控器 App 层：初始化 + ISR 回调发布 + Daemon 离线检测。

> `Remote` 模块在 UART ISR 中解码并通过回调直接发布到 `remote_topic`，零轮询延迟。
> 离线检测由 `DaemonInstance` 自动完成（100 Hz TickAll 驱动），无需手动轮询。

## 工作流程

### 初始化 (RemoteInit)

1. 创建 `DaemonInstance`：10 tick @ 100 Hz = 100ms 超时，离线回调重启 UART
2. 创建 `Remote<Dt7Protocol>`：注入发布回调（Publish + Reload 喂狗）
3. UART DMA+IDLE 接收自动启动

### 数据路径（ISR 回调直接发布）

```
UART DMA Complete ISR
  → HAL_UARTEx_RxEventCallback
    → sal::UartInstance rx_cbk_
      → Remote::OnReceive()
        → Dt7Protocol::Decode()
        → SeqLock 写快照
        → on_publish_(curr)
            → remote_topic.Publish(d)   // 唯一写者
            → rc_daemon->Reload()       // 喂狗 (单次 32-bit 原子写)
```

`remote_topic` 只有 ISR 回调一个写者，满足 Topic 单生产者约束。

### 离线检测（Daemon FSM）

由 `daemon_task` 以 100 Hz 调用 `DaemonInstance::TickAll()` 驱动：

```
UNSEEN ──(首次 Reload)──→ ONLINE ──(100ms 超时)──→ OFFLINE
                            ↑                        │
                            └──(再次 Reload)──────────┘
```

- **ONLINE → OFFLINE**: 触发 `on_offline` 回调，执行 `RestartRx()` 重启 UART
- **OFFLINE → ONLINE**: 遥控器恢复数据后自动回到 ONLINE
- **消费者侧安全**: cmd_task 通过 `RemoteIsOnline()` 判断，离线时使用零值数据

Daemon 回调不写 Topic（避免双写者竞态），安全策略由消费者负责。

## 外部接口

```cpp
void RemoteInit();          // 由 RobotInit() 调用
bool RemoteIsOnline();      // 消费者查询遥控器在线状态
Dt7Remote* GetRemote();     // 调试用途
```
