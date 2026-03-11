# remote

遥控器模块，使用模板策略 `Remote<Protocol>` 实现零虚函数多态。

## 总览

`Remote` 是 UART 回调驱动的数据源：

```
UART DMA ISR → Remote::OnReceive()
             → Protocol::Decode()
             → 更新内部快照(data_ + seq_)
```

模块不持有 Topic，不做发布决策。App 层通过 bridge/pump 调用 `ReadSnapshot()` 获取数据后发布。

## Protocol 契约

```cpp
struct SomeProtocol {
    static constexpr uint16_t FRAME_SIZE = ...;
    using Data = SomeData;  // trivially copyable
    static void Decode(const uint8_t* buf, Data& curr, const Data& prev);
    static void Reset(Data& data);
};
```

## Remote<Protocol> 接口

```cpp
Remote(const RemoteConfig& cfg);
bool ReadSnapshot(Data& out, uint32_t* seq_out = nullptr) const;
uint32_t SnapshotSeq() const;
void RestartRx();
```

- `ReadSnapshot()` 使用轻量 SeqLock 读一致快照，若被 ISR 写打断则返回 `false`。
- `SnapshotSeq()` 可用于 app 层判断是否有新帧。

## DT7/DR16

协议为 DBUS（100000 baud, 8E1, 18-byte frame, ~70Hz），解析类型为 `Dt7Data`。

## 使用范例

```cpp
remote::RemoteConfig cfg{};
cfg.uart_handle = &huart3;
auto* rc = new remote::Remote<remote::Dt7Protocol>(cfg);

remote::Dt7Data data;
uint32_t seq = 0;
if (rc->ReadSnapshot(data, &seq)) {
    // app bridge decides whether to publish
}
```
