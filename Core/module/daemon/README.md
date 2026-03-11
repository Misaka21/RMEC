# daemon

确定性 Health Monitor，用于监测模块在线状态。基于 FSM + 时间戳模型，替代传统倒计数方案。

## 代码结构

| 文件 | 说明 |
|---|---|
| `daemon.hpp` | header-only，`DaemonInstance` 类 + `DaemonConfig` + `State` 枚举 |

## 设计要点

### FSM 状态机

```
UNSEEN ──(首次 Reload)──→ ONLINE ──(超时)──→ OFFLINE
                            ↑                   │
                            └──(再次 Reload)─────┘
```

| 迁移 | 条件 | 回调 |
|---|---|---|
| UNSEEN → ONLINE | 首次 `Reload()` | 无 (静默上线) |
| ONLINE → OFFLINE | `current_tick_ - last_feed_tick_ > timeout_ticks_` | `on_offline(owner)` |
| OFFLINE → ONLINE | 新的 `Reload()` 使时间差回到阈值内 | `on_recover(owner)` |

- **边沿触发**: 回调仅在状态迁移时触发一次，非电平重复触发
- **OFFLINE 锁存**: 超时后持续 OFFLINE，直到模块真正恢复 `Reload()`，`IsOnline()` 语义准确

### 时间戳模型 (vs 倒计数)

传统倒计数 (`--count`) 是 RMW 操作，ISR 写入与 TickAll 递减存在竞态。

时间戳模型分离读写：
- **ISR 端**: `Reload()` 只做 `last_feed_tick_ = current_tick_`（单次 32-bit 原子写，零竞态）
- **TickAll 端**: 只读 `last_feed_tick_`，独占 `state_` 和计时比较

`current_tick_` 从 1 起步，0 作为"未喂过"哨兵值。

### 自注册

构造时自动加入固定容量静态表 (`MAX_INSTANCES = 32`)，无需手动注册。模块内部零堆分配（实例由上层决定静态或堆分配）。与 SAL 层实例管理模式一致。

## 配置

```cpp
struct DaemonConfig {
    uint16_t timeout_ticks = 100;       // 超时阈值 (TickAll 周期数)
    void(*on_offline)(void*) = nullptr; // ONLINE → OFFLINE 回调
    void(*on_recover)(void*) = nullptr; // OFFLINE → ONLINE 回调
    void* owner = nullptr;              // 回调上下文 (模块实例指针)
};
```

`timeout_ticks` 根据 `TickAll()` 调用频率和模块数据频率设定。例如 TickAll 以 100 Hz 运行时，`timeout_ticks = 10` 表示 100ms 超时。

## 外部接口

```cpp
DaemonInstance(const DaemonConfig& cfg); // 构造即注册
void Reload();                           // 喂狗 (ISR 安全)
bool IsOnline() const;                   // 查询是否在线
State GetState() const;                  // 查询 FSM 状态
static void TickAll();                   // 周期 tick 所有实例
```

## 使用范例

```cpp
#include "daemon.hpp"

// 创建 daemon: 100Hz TickAll, 10 tick = 100ms 超时
auto* daemon = new daemon::DaemonInstance({
    .timeout_ticks = 10,
    .on_offline = [](void*) { /* 重启接收等恢复操作 */ },
});

// ISR 或数据接收回调中喂狗
void OnDataReceived() {
    daemon->Reload();
}

// App 层 100Hz 任务中调用
void DaemonTaskLoop() {
    daemon::DaemonInstance::TickAll();
}

// 消费者侧检查在线状态
if (daemon->IsOnline()) {
    // 使用模块数据
}
```

## 与 basic_framework 的区别

| 项目 | basic_framework | powerful_framework |
|---|---|---|
| 计时模型 | 倒计数 (`--count`)，ISR 写 + Task 减有 RMW 竞态 | 时间戳比较，ISR 只写，TickAll 只读 |
| 状态模型 | 隐式 (count == 0 即离线) | 显式 FSM (UNSEEN/ONLINE/OFFLINE) |
| 回调类型 | 仅离线回调 | 离线 + 恢复双边沿回调 |
| 内存 | `malloc` + 链表 | 固定数组，零堆分配 |
| 回调签名 | `void(*)(void*)` | 同 (ISR 零开销) |
