# Daemon 守护进程

<p align='right'>powerful_framework</p>

> 确定性健康监测模块，FSM + 时间戳模型，零竞态设计

---

## 如果你只需要快速使用

```cpp
#include "daemon/daemon.hpp"

// 1. 创建守护实例（检测 UART 离线）
DaemonInstance daemon({
    .timeout_ticks = 100,  // 100 个 tick 周期
    .on_offline = [](void* owner) {
        auto* uart = static_cast<UartInstance*>(owner);
        uart->Restart();  // 离线时重启 UART
    },
    .on_recover = [](void* owner) {
        LOGINFO("UART back online!");
    },
    .owner = uart_instance  // 传递给回调的上下文
});

// 2. 收到数据时喂狗
void OnUartReceive(uint8_t* buf, uint16_t len) {
    ProcessData(buf, len);
    daemon.Reload();  // ISR 安全，单次原子写
}

// 3. 创建守护任务（推荐 100Hz）
void DaemonTask() {
    while (true) {
        DaemonInstance::TickAll();  // 轮询所有守护实例
        vTaskDelay(10);  // 10ms = 100Hz
    }
}
```

---

## 状态机模型

```
        ┌─────────┐
        │  UNSEEN │◄──── 初始状态（未收到过喂狗）
        └────┬────┘
             │ Reload (首次)
             ▼
        ┌─────────┐     timeout      ┌─────────┐
        │  ONLINE │─────────────────►│ OFFLINE │
        └────┬────┘                  └────┬────┘
             ▲                           │
             │ Reload                    │ Reload
             └───────────────────────────┘
```

### 状态说明

| 状态 | 含义 | 触发条件 |
|------|------|----------|
| `UNSEEN` | 注册后未收到过数据 | 初始状态 |
| `ONLINE` | 正常在线 | 首次 Reload 后 |
| `OFFLINE` | 超时离线 | 超过 timeout_ticks 未 Reload |

### 边沿触发回调

- `ONLINE → OFFLINE`: 触发 `on_offline`
- `OFFLINE → ONLINE`: 触发 `on_recover`

---

## 配置参数

```cpp
struct DaemonConfig {
    uint16_t timeout_ticks = 100;       // 超时阈值
    void(*on_offline)(void*) = nullptr; // 离线回调
    void(*on_recover)(void*) = nullptr; // 恢复回调
    void* owner = nullptr;              // 回调上下文指针
};
```

### timeout_ticks 计算

```
实际超时时间 = timeout_ticks × TickAll 调用周期

示例:
- timeout_ticks = 100
- TickAll 周期 = 10ms (100Hz)
- 实际超时 = 100 × 10ms = 1000ms = 1s
```

### 推荐配置

| 模块 | 数据频率 | timeout_ticks | TickAll 周期 | 实际超时 |
|------|---------|---------------|--------------|----------|
| 遥控器 | 1000Hz | 50 | 10ms | 500ms |
| 电机反馈 | 500Hz | 100 | 10ms | 1s |
| 视觉通信 | 100Hz | 20 | 10ms | 200ms |
| 裁判系统 | 100Hz | 30 | 10ms | 300ms |

---

## 并发安全模型

### 零竞态设计

```
ISR 上下文（高优先级）:
    Reload() ──► last_feed_tick_ = current_tick_  (单次 32-bit 写)

Task 上下文（低优先级）:
    TickAll() ──► 读取 last_feed_tick_ ──► 状态机更新
```

- **ISR 只写**，单次 32-bit 原子写，无 RMW（读-改-写）
- **Task 只读**，不阻塞 ISR
- **无锁化**，适合实时系统

### 注意事项

1. `Reload()` 可在 ISR 安全调用
2. `IsOnline()` 任意上下文均可调用
3. `TickAll()` 必须在 Task 上下文调用（会遍历所有实例）

---

## 完整示例：电机离线保护

```cpp
// motor_task.cpp
#include "daemon/daemon.hpp"
#include "motor/motor.hpp"
#include "motor/driver/dji_driver.hpp"
#include "motor/controller/cascade_pid.hpp"

using MotorType = Motor<DjiDriver, CascadePid>;

struct MotorEntry {
    MotorType* motor;
    DaemonInstance* daemon;
};

MotorEntry motors[4];

void MotorInit() {
    // 配置 4 个底盘电机
    for (int i = 0; i < 4; i++) {
        DjiDriverConfig d_cfg = {
            .motor_type = DjiMotorType::M3508,
            .can_handle = &hcan2,
            .motor_id = i + 1
        };
        CascadePidConfig c_cfg = {
            .speed_pid = {10.0f, 0.5f, 0.0f, 20.0f},
            .loop_mode = loop_mode::SPEED
        };

        auto* motor = new MotorType(d_cfg, c_cfg);

        // 每个电机配一个守护
        auto* daemon = new DaemonInstance({
            .timeout_ticks = 100,  // 1s 超时
            .on_offline = [](void* owner) {
                auto* m = static_cast<MotorType*>(owner);
                m->Disable();  // 离线停止
                LOGWARN("Motor offline!");
            },
            .on_recover = [](void* owner) {
                auto* m = static_cast<MotorType*>(owner);
                m->Enable();   // 恢复上线
                LOGINFO("Motor back online!");
            },
            .owner = motor
        });

        motors[i] = {motor, daemon};
        motor->Enable();
    }
}

void MotorTask() {
    while (true) {
        for (auto& entry : motors) {
            auto* m = entry.motor;
            auto* d = entry.daemon;

            // 控制循环
            if (m->IsOnline()) {
                m->Update(0.001f);
            }

            // 收到 CAN 反馈时喂狗（在 CAN 回调中）
            // d->Reload();
        }

        DjiDriver::FlushAll();
        vTaskDelay(1);  // 1kHz
    }
}

// CAN 接收回调（ISR 上下文）
void OnCanFeedback(uint8_t* data) {
    // ... 解析数据 ...

    // 找到对应电机喂狗
    motors[id].daemon->Reload();
}
```

---

## 全局守护任务

```cpp
// daemon_task.cpp
#include "daemon/daemon.hpp"

void DaemonTaskStart() {
    xTaskCreate([](void*) {
        while (true) {
            DaemonInstance::TickAll();
            vTaskDelay(10);  // 100Hz
        }
    }, "DaemonTask", 256, nullptr, 3, nullptr);
}
```

---

## 与 basic_framework 的区别

| 特性 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| 模型 | 倒计时计数器 | FSM + 时间戳 |
| 状态 | ONLINE/OFFLINE | UNSEEN/ONLINE/OFFLINE |
| 回调 | 仅 offline | offline + recover |
| 并发 | count-- 非原子 | 32-bit tick 原子写 |
| 自动重试 | 无 | OFFLINE 状态锁存，需新 Reload |

---

## 注意事项

1. **首次 Reload**: UNSEEN → ONLINE 是静默的，不触发 on_recover

2. **离线锁存**: OFFLINE 状态会一直保持，直到新的 Reload，不会自动恢复

3. **回调限制**: `on_offline` 和 `on_recover` 在 TickAll 上下文调用，不要阻塞

4. **最大实例数**: `MAX_INSTANCES = 32`，超过会忽略

5. **TickAll 频率**: 推荐 100Hz，影响实际超时精度
