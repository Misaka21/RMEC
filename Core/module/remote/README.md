# remote

遥控器模块，使用模板策略 `Remote<Protocol>` 实现零虚函数多态。

## 总览

`Remote` 是 UART ISR 回调驱动的数据源，支持通过 `PublishCallback` 在 ISR 上下文直接发布：

```
UART DMA ISR → Remote::OnReceive()
             → Protocol::Decode()
             → SeqLock 写快照 (data_ + seq_)
             → on_publish_(curr)    // App 注入的发布回调
```

模块不持有 Topic。App 层通过构造时注入的回调决定发布策略。

## Protocol 契约

```cpp
struct SomeProtocol {
    static constexpr uint16_t FRAME_SIZE = ...;
    using Data = SomeData;  // trivially copyable
    static void Decode(const uint8_t* buf, Data& curr, const Data& prev);
    static void Reset(Data& data);
};
```

编译期通过 `CheckProtocolInterface<P>()` 静态断言检查契约。

## Remote<Protocol> 接口

```cpp
Remote(const RemoteConfig& cfg, PublishCallback on_publish = nullptr);
bool ReadSnapshot(Data& out, uint32_t* seq_out = nullptr) const;
uint32_t SnapshotSeq() const;
void RestartRx();
```

- `PublishCallback` 是裸函数指针 `void(*)(const Data&)`，ISR 零开销
- `on_publish_` 在 SeqLock 写完成后调用，是 Topic 的唯一写者路径
- `ReadSnapshot()` 使用轻量 SeqLock 读一致快照，若被 ISR 写打断则返回 `false`
- `RestartRx()` 重启 UART DMA 接收（供 Daemon 离线回调使用）

## DT7/DR16

协议为 DBUS（100000 baud, 8E1, 18-byte frame, ~70Hz），解析类型为 `Dt7Data`。

## 使用范例

```cpp
remote::RemoteConfig cfg{};
cfg.uart_handle = &huart3;

// ISR 回调直接发布到 Topic + 喂狗
auto* rc = new remote::Remote<remote::Dt7Protocol>(cfg,
    [](const remote::Dt7Data& d) {
        remote_topic.Publish(d);
        rc_daemon->Reload();
    });
```
