# can_comm

双板 CAN 通信模块, 使用多 ID 映射实现任意长度结构体透传。

## 总览

`CanComm<TxData, RxData>` 是 header-only 模板, 将发送/接收结构体按 8 字节切片, 每片分配独立 CAN ID:

```
发送 struct (例如 24 bytes):
  ┌──────────┬──────────┬──────────┐
  │ byte 0-7 │ byte 8-15│ byte16-23│
  └────┬─────┴────┬─────┴────┬─────┘
       │          │          │
    tx_id+0    tx_id+1    tx_id+2    ← 每帧独立发送
```

- `kTxFrames = ceil(sizeof(TxData) / 8)`, 编译期自动计算
- `kRxFrames = ceil(sizeof(RxData) / 8)`, 编译期自动计算
- 无帧头/帧尾/CRC/状态机, 每帧独立, 零额外开销

## 代码结构

| 文件 | 说明 |
|---|---|
| `can_comm.hpp` | header-only, `CanComm<TxData, RxData>` 模板 + `CanCommConfig` |

## 配置

```cpp
struct CanCommConfig {
    CAN_HandleTypeDef* can_handle;
    uint16_t base_tx_id;         // 发送基础 ID (连续分配 kTxFrames 个)
    uint16_t base_rx_id;         // 接收基础 ID (连续分配 kRxFrames 个)
    uint16_t daemon_timeout = 0; // 0 = 不创建 Daemon
};
```

ID 分配规则: 发送占用 `[base_tx_id, base_tx_id + kTxFrames)`, 接收占用 `[base_rx_id, base_rx_id + kRxFrames)`。需确保与电机反馈 ID (0x201-0x20B) 无交叉。

## 外部接口

```cpp
explicit CanComm(const CanCommConfig& cfg);
void Send(const TxData& data);          // 切片发送 N 帧 (task context)
const RxData& Recv() const;             // 返回最新接收数据引用
bool IsOnline() const;                  // Daemon 在线判断
```

## 发送流程

`Send()` 将 TxData `memcpy` 切片为 N 帧, 每帧通过对应 `CanInstance` 的 `CanTransmit()` 独立发送。最后一帧长度可能不足 8 字节, 自动计算实际拷贝长度。

## 接收流程

构造时为前 `kRxFrames` 个 `CanInstance` 注册 ISR 回调:

1. 按帧索引 `i` 计算 `rx_buf_` 中的偏移 `i * 8`
2. `memcpy` 从 `CanInstance::RxData()` 写入对应位置
3. 每帧到达都调用 `Daemon::Reload()` 喂狗

接收无需所有帧同时到达, 各帧独立更新。

## 类型约束

- `TxData` 和 `RxData` 必须满足 `std::is_trivially_copyable_v`
- 建议使用 `#pragma pack(1)` 避免 padding 导致的带宽浪费

## 使用范例

```cpp
#include "can_comm.hpp"

#pragma pack(1)
struct MyTxData { float vx; float vy; uint8_t mode; };
struct MyRxData { float yaw; float pitch; };
#pragma pack()

CanCommConfig cfg{};
cfg.can_handle     = &hcan1;
cfg.base_tx_id     = 0x100;
cfg.base_rx_id     = 0x110;
cfg.daemon_timeout = 100;

auto* comm = new CanComm<MyTxData, MyRxData>(cfg);

// 发送
MyTxData tx{1.0f, 2.0f, 1};
comm->Send(tx);

// 接收
const MyRxData& rx = comm->Recv();
if (comm->IsOnline()) {
    // 使用 rx.yaw, rx.pitch
}
```

## 与 basic_framework 的区别

| 项目 | basic_framework | powerful_framework |
|---|---|---|
| 传输方式 | 单 ID 分包 + 帧头帧尾状态机 | 多 ID 映射, 每帧独立 |
| 协议开销 | 帧头 + 长度 + CRC + 帧尾 (4 字节/包) | 零开销 |
| 丢帧影响 | 丢一帧整包废弃 | 仅丢失帧对应字段, 其余不受影响 |
| 最大长度 | 60 字节 (可调) | 编译期自动, 无硬限制 |
| 类型安全 | void* + sizeof | 模板参数, 编译期检查 |
