# CanComm 双板通信

<p align='right'>powerful_framework</p>

> 模板化的 CAN 总线双板通信，自动切片/重组，SeqLock 无锁读取

---

## 如果你只需要快速使用

```cpp
#include "can_comm/can_comm.hpp"

// 1. 定义通信数据结构
struct BoardCommTxData {
    float chassis_vx;
    float chassis_vy;
    float chassis_wz;
    uint8_t chassis_mode;
    uint8_t reserved[3];
};

struct BoardCommRxData {
    float gimbal_yaw;
    float gimbal_pitch;
    uint8_t fire_command;
    uint8_t reserved[3];
};

// 2. 创建 CanComm 实例
CanComm<BoardCommTxData, BoardCommRxData> comm({
    .can_handle = &hcan1,
    .base_tx_id = 0x100,    // 发送 ID: 0x100, 0x101, ...
    .base_rx_id = 0x110,    // 接收 ID: 0x110, 0x111, ...
    .daemon_timeout = 50    // 500ms 离线检测
});

// 3. 发送数据
void SendData() {
    BoardCommTxData tx = {
        .chassis_vx = 1.5f,
        .chassis_vy = 0.0f,
        .chassis_wz = 0.5f,
        .chassis_mode = 1
    };
    comm.Send(tx);
}

// 4. 接收数据
void ReceiveData() {
    BoardCommRxData rx;
    if (comm.Recv(rx)) {
        // 处理接收到的数据
        float yaw = rx.gimbal_yaw;
    }
}
```

---

## 自动切片机制

CanComm 自动将大数据结构切分为多个 CAN 帧：

```
sizeof(TxData) = 24 字节
              ↓
        ┌─────┬─────┬─────┐
        │ 8B  │ 8B  │ 8B  │  → 3 个 CAN 帧
        │ 0x100│0x101│0x102│
        └─────┴─────┴─────┘
```

### 帧数计算

```cpp
static constexpr int kTxFrames = (sizeof(TxData) + 7) / 8;  // 向上取整
static constexpr int kRxFrames = (sizeof(RxData) + 7) / 8;
```

### ID 分配

| 帧序号 | TX ID | RX ID |
|--------|-------|-------|
| 0 | base_tx_id + 0 | base_rx_id + 0 |
| 1 | base_tx_id + 1 | base_rx_id + 1 |
| 2 | base_tx_id + 2 | base_rx_id + 2 |
| ... | ... | ... |

---

## 接收机制

### 位掩码重组

```
帧 0 到达: rx_mask = 0b001
帧 1 到达: rx_mask = 0b011
帧 2 到达: rx_mask = 0b111  ← 全部到齐，SeqLock 提交
```

### 帧同步

- 帧 0 到达时清除未完成的数据（防止新旧数据混合）
- 所有帧到齐后通过 SeqLock 原子提交

```cpp
if (i == 0 && rx_mask_ != 0) {
    // 新一轮开始，清除旧数据
    memset(&rx_staging_, 0, sizeof(rx_staging_));
    rx_mask_ = 0;
}
```

---

## 完整示例：底盘-云台双板通信

```cpp
// robot_topics.hpp
#pragma once
#include "can_comm/can_comm.hpp"
#include "topic.hpp"

// 云台→底盘数据
struct Gimbal2ChassisData {
    float yaw;           // 云台 yaw 角度
    float pitch;         // 云台 pitch 角度
    float target_yaw;    // 目标角度
    uint8_t gimbal_mode; // 云台模式
    uint8_t fire_flag;   // 发射标志
    uint8_t reserved[2];
};

// 底盘→云台数据
struct Chassis2GimbalData {
    float vx, vy, wz;         // 底盘速度
    float power_buffer;       // 缓冲能量
    uint16_t remain_hp;       // 剩余血量
    uint8_t chassis_mode;     // 底盘模式
    uint8_t super_cap_status; // 超电状态
};

// Topic 定义
inline Topic<Gimbal2ChassisData> gimbal2chassis_topic;
inline Topic<Chassis2GimbalData> chassis2gimbal_topic;

// CanComm 实例（定义在 comm_task.cpp）
extern CanComm<Chassis2GimbalData, Gimbal2ChassisData>* board_comm;
```

```cpp
// comm_task.cpp（底盘板）
#include "can_comm/can_comm.hpp"
#include "robot_topics.hpp"

CanComm<Chassis2GimbalData, Gimbal2ChassisData>* board_comm = nullptr;

void CommTaskStart() {
    // 创建 CanComm（底盘板）
    board_comm = new CanComm<Chassis2GimbalData, Gimbal2ChassisData>({
        .can_handle = &hcan1,
        .base_tx_id = 0x100,  // 底盘发送
        .base_rx_id = 0x110,  // 底盘接收
        .daemon_timeout = 50  // 500ms 超时
    });

    xTaskCreate([](void*) {
        Chassis2GimbalData tx_data{};
        Gimbal2ChassisData rx_data;

        while (true) {
            // 读取底盘数据
            auto chassis = GetChassisState();
            tx_data.vx = chassis.vx;
            tx_data.vy = chassis.vy;
            tx_data.wz = chassis.wz;
            tx_data.power_buffer = chassis.power_buffer;

            // 发送到底盘
            board_comm->Send(tx_data);

            // 接收云台数据并发布到 Topic
            if (board_comm->Recv(rx_data)) {
                gimbal2chassis_topic.Publish(rx_data);
            }

            vTaskDelay(10);  // 100Hz
        }
    }, "CommTask", 512, nullptr, 2, nullptr);
}
```

```cpp
// comm_task.cpp（云台板）
void CommTaskStart() {
    // 创建 CanComm（云台板）- ID 对调
    board_comm = new CanComm<Gimbal2ChassisData, Chassis2GimbalData>({
        .can_handle = &hcan1,
        .base_tx_id = 0x110,  // 云台发送（对应底盘接收）
        .base_rx_id = 0x100,  // 云台接收（对应底盘发送）
        .daemon_timeout = 50
    });

    xTaskCreate([](void*) {
        Gimbal2ChassisData tx_data{};
        Chassis2GimbalData rx_data;

        while (true) {
            // 读取云台数据
            auto gimbal = GetGimbalState();
            tx_data.yaw = gimbal.yaw;
            tx_data.pitch = gimbal.pitch;

            // 发送
            board_comm->Send(tx_data);

            // 接收底盘数据
            if (board_comm->Recv(rx_data)) {
                chassis2gimbal_topic.Publish(rx_data);
            }

            vTaskDelay(10);
        }
    }, "CommTask", 512, nullptr, 2, nullptr);
}
```

---

## 在线检测

```cpp
// 查询通信是否在线
if (comm.IsOnline()) {
    // 双板通信正常
} else {
    // 通信断开，使用默认行为
}
```

内部使用 Daemon 检测，超时未收到完整帧视为离线。

---

## 数据对齐要求

```cpp
// 使用 #pragma pack 确保紧凑布局，无填充字节
#pragma pack(push, 1)
struct MyData {
    float x;      // 4B
    uint8_t flag; // 1B
    // 编译器可能插入 3B 填充！
};
#pragma pack(pop)
```

**务必使用 `#pragma pack(1)` 确保结构体紧凑！**

---

## 与 basic_framework 的区别

| 特性 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| 协议 | 自定义帧头+CRC | 无帧头，纯切片 |
| 同步 | 帧序号+状态机 | 位掩码+SeqLock |
| 数据对齐 | 手动处理 | 模板自动处理 |
| 离线检测 | 单独实现 | 内置 Daemon |
| 读取 | 全局变量 | SeqLock + Topic |

---

## 注意事项

1. **数据大小**: 单帧 8B，大数据结构会被切分为多帧，频率高时占用 CAN 带宽

2. **ID 冲突**: 确保 base_tx_id/base_rx_id 与其他 CAN 设备不冲突

3. **接收超时**: 部分帧丢失会导致整包丢弃（帧 0 到达时重置）

4. **字节对齐**: 务必使用 `#pragma pack(1)`，否则可能出错

5. **双向通信**: 两板的 TX/RX ID 必须对调，否则收不到数据
