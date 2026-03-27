# 遥控器模块 (Remote)

<p align='right'>powerful_framework</p>

> 模板策略模式设计的遥控器驱动，支持 DT7 等多种协议

---

## 如果你只需要快速使用

```cpp
#include "remote/remote.hpp"
#include "remote/protocol/dt7_protocol.hpp"
#include "app/robot_topics.hpp"

void RemoteTaskStart() {
    // 创建 DT7 遥控器，ISR 回调直接发布到 Topic
    static remote::Remote<remote::Dt7Protocol> remote_rc({
        .uart_handle = &huart3
    }, [](const remote::Dt7Data& data) {
        remote_topic.Publish(data);
    });

    // 创建守护进程监控离线
    static DaemonInstance daemon({
        .timeout_ticks = 50,  // 500ms 超时
        .on_offline = [](void*) { /* 离线处理 */ },
        .owner = &remote_rc
    });

    // 守护任务中喂狗
    xTaskCreate([](void*) {
        while (true) {
            remote::Dt7Data data;
            if (remote_rc.ReadSnapshot(data)) {
                daemon.Reload();  // 收到数据就喂狗
            }
            vTaskDelay(10);
        }
    }, "RemoteDaemon", 256, nullptr, 2, nullptr);
}
```

---

## 架构设计

```
Remote<Protocol>
      │
      ▼
┌─────────────┐
│ UartInstance│  DMA_IDLE 接收
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  Protocol   │  Decode(buf, curr, prev)
│  ::Decode   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  SeqLock    │  ISR 安全写
│  on_publish │  回调 → Topic
└─────────────┘
```

### 零虚函数设计

- `Protocol` 是模板参数，编译期确定
- 无虚表开销，适合 ISR 高频调用
- 支持自定义协议（SBUS、CRSF 等）

---

## DT7 协议详解

### 数据帧结构

DBUS 协议，18 字节固定长度，波特率 100000（注意不是 115200）

```
字节: [0-17] 共 18 字节
```

### 通道映射

```cpp
struct Dt7Data {
    // 摇杆 (0-2047, 中值 1024)
    uint16_t right_x;   // 通道 0: 右摇杆左右
    uint16_t right_y;   // 通道 1: 右摇杆上下
    uint16_t left_x;    // 通道 2: 左摇杆左右
    uint16_t left_y;    // 通道 3: 左摇杆上下

    // 拨杆开关 (1=上, 3=中, 2=下)
    SwitchPos switch_right;  // 通道 4: 右侧开关 SW_R
    SwitchPos switch_left;   // 通道 5: 左侧开关 SW_L

    // 鼠标 (有符号 16 位)
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;   // 0/1
    uint8_t mouse_right;  // 0/1

    // 键盘 (位图，每个 bit 代表一个键)
    uint16_t keyboard;

    // 拨轮 (0-2047)
    uint16_t dial;
};
```

### 开关位置枚举

```cpp
enum class SwitchPos : uint8_t {
    UP    = 1,  // 上
    MID   = 3,  // 中
    DOWN  = 2   // 下
};
```

### 键盘位定义

```cpp
enum class Dt7Key : uint16_t {
    W = 0x0001,
    S = 0x0002,
    A = 0x0004,
    D = 0x0008,
    SHIFT = 0x0010,
    CTRL  = 0x0020,
    Q = 0x0040,
    E = 0x0080,
    R = 0x0100,
    F = 0x0200,
    G = 0x0400,
    Z = 0x0800,
    X = 0x1000,
    C = 0x2000,
    V = 0x4000,
    B = 0x8000,
};
```

### 按键检测

```cpp
void HandleKeyboard(const Dt7Data& data) {
    // 检测 W 键是否按下
    if (data.keyboard & static_cast<uint16_t>(Dt7Key::W)) {
        // W 键被按下
    }

    // 检测组合键
    uint16_t ctrl_a = static_cast<uint16_t>(Dt7Key::CTRL)
                    | static_cast<uint16_t>(Dt7Key::A);
    if ((data.keyboard & ctrl_a) == ctrl_a) {
        // Ctrl+A 组合
    }
}
```

---

## 协议契约

自定义协议需满足以下接口：

```cpp
struct MyProtocol {
    // 帧大小（字节）
    static constexpr uint16_t FRAME_SIZE = 25;

    // 数据类型（必须 trivially copyable）
    struct Data {
        float x, y, z;
        // ...
    };

    // 解码函数
    static void Decode(const uint8_t* buf, Data& curr, const Data& prev);

    // 重置数据
    static void Reset(Data& data);
};
```

### 使用自定义协议

```cpp
remote::Remote<MyProtocol> my_remote({.uart_handle = &huart2});
```

---

## 完整示例：遥控任务

```cpp
// remote_task.cpp
#include "remote/remote.hpp"
#include "remote/protocol/dt7_protocol.hpp"
#include "daemon/daemon.hpp"
#include "app/robot_topics.hpp"

using namespace remote;

static Remote<Dt7Protocol>* rc = nullptr;
static DaemonInstance* rc_daemon = nullptr;

void RemoteTaskStart() {
    // 创建遥控器实例
    rc = new Remote<Dt7Protocol>({
        .uart_handle = &huart3
    }, [](const Dt7Data& data) {
        // ISR 回调：直接发布到 Topic
        remote_topic.Publish(data);
    });

    // 创建守护进程
    rc_daemon = new DaemonInstance({
        .timeout_ticks = 50,  // 500ms
        .on_offline = [](void*) {
            // 遥控器离线：发送停止命令
            EmergencyStop();
        },
        .owner = rc
    });

    // 创建守护任务（100Hz）
    xTaskCreate([](void*) {
        Dt7Data data;
        while (true) {
            // 读取最新数据
            if (rc->ReadSnapshot(data)) {
                rc_daemon->Reload();  // 喂狗
            }
            vTaskDelay(10);  // 10ms = 100Hz
        }
    }, "RcDaemon", 256, nullptr, 2, nullptr);
}

// 供其他任务查询
bool RemoteIsOnline() {
    return rc_daemon && rc_daemon->IsOnline();
}

const Dt7Data* GetRemoteData() {
    static Dt7Data data;
    if (rc && rc->ReadSnapshot(data)) {
        return &data;
    }
    return nullptr;
}
```

---

## 键鼠控制模式解析

### 常用映射参考

```cpp
void ProcessCommand(const Dt7Data& rc) {
    // 左侧开关控制模式
    switch (rc.switch_left) {
        case SwitchPos::UP:
            // 键鼠控制模式
            ProcessKeyboardMouse(rc);
            break;
        case SwitchPos::MID:
            // 视觉控制模式
            ProcessVisionMode(rc);
            break;
        case SwitchPos::DOWN:
            // 遥控器控制模式
            ProcessRcMode(rc);
            break;
    }

    // 右侧开关控制功能
    switch (rc.switch_right) {
        case SwitchPos::UP:
            // 弹舱开
            OpenBulletCover();
            break;
        case SwitchPos::MID:
            // 底盘云台分离
            SetChassisFollowGimbal(false);
            break;
        case SwitchPos::DOWN:
            // 底盘跟随云台
            SetChassisFollowGimbal(true);
            break;
    }

    // 拨轮控制
    if (rc.dial > 1500) {
        StartFrictionWheel();
    } else {
        StopFrictionWheel();
    }
}

void ProcessKeyboardMouse(const Dt7Data& rc) {
    // WASD 控制底盘移动
    float vx = 0, vy = 0;
    if (rc.keyboard & static_cast<uint16_t>(Dt7Key::W)) vy += 1.0f;
    if (rc.keyboard & static_cast<uint16_t>(Dt7Key::S)) vy -= 1.0f;
    if (rc.keyboard & static_cast<uint16_t>(Dt7Key::A)) vx -= 1.0f;
    if (rc.keyboard & static_cast<uint16_t>(Dt7Key::D)) vx += 1.0f;

    // 鼠标控制云台
    float yaw_delta   = rc.mouse_x * 0.001f;
    float pitch_delta = rc.mouse_y * 0.001f;

    // Shift 加速
    float speed_factor = (rc.keyboard & static_cast<uint16_t>(Dt7Key::SHIFT))
                         ? 2.0f : 1.0f;

    SetChassisSpeed(vx * speed_factor, vy * speed_factor);
    SetGimbalDelta(yaw_delta, pitch_delta);
}
```

---

## 注意事项

1. **波特率**: DT7 使用 100000，不是标准 115200

2. **单写者约束**: Remote 的 ISR 回调是 remote_topic 的唯一写者，daemon 只喂狗不写 Topic

3. **离线检测**: 务必配合 Daemon 使用，检测遥控器断连

4. **SeqLock 读取**: `ReadSnapshot()` 可能返回 false（正在写入），消费者应处理失败情况

5. **DMA 缓冲区**: UART 接收缓冲区大小 = FRAME_SIZE，确保 DMA 配置正确
