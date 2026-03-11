#pragma once

#include <cstdint>

namespace daemon {

inline constexpr uint8_t MAX_INSTANCES = 32;

// ======================== 状态 ========================

enum class State : uint8_t {
    UNSEEN,   // 注册后未收到过数据 (开机初始)
    ONLINE,   // 正常在线
    OFFLINE,  // 超时离线 (锁存, 直到下次 Reload)
};

// ======================== 配置 ========================

struct DaemonConfig {
    uint16_t timeout_ticks = 100;           // 超时阈值 (TickAll 周期数)
    void(*on_offline)(void*) = nullptr;     // ONLINE → OFFLINE 边沿
    void(*on_recover)(void*) = nullptr;     // OFFLINE → ONLINE 边沿
    void* owner = nullptr;                  // 回调上下文 (模块实例指针)
};

// ======================== DaemonInstance ========================
/// 确定性 Health Monitor (FSM + 时间戳模型)
///
/// 状态迁移 (仅 TickAll 驱动, 边沿触发):
///   UNSEEN  → ONLINE   首次 Reload (静默, 无回调)
///   ONLINE  → OFFLINE  超时 (触发 on_offline)
///   OFFLINE → ONLINE   再次 Reload (触发 on_recover)
///
/// 并发模型:
///   ISR 只写 last_feed_tick_ (单次 32-bit 原子写, 零竞态)
///   TickAll 只读 last_feed_tick_, 独占 state_ 和计时比较
///
/// 自注册: 构造时加入固定容量静态表

class DaemonInstance {
public:
    explicit DaemonInstance(const DaemonConfig& cfg)
        : timeout_ticks_(cfg.timeout_ticks),
          on_offline_(cfg.on_offline),
          on_recover_(cfg.on_recover),
          owner_(cfg.owner)
    {
        if (count_ < MAX_INSTANCES) {
            instances_[count_++] = this;
        }
    }

    /// 喂狗 (ISR 安全: 单次 32-bit 原子写, 无 RMW)
    void Reload() { last_feed_tick_ = current_tick_; }

    /// 状态查询
    bool IsOnline() const { return state_ == State::ONLINE; }
    State GetState() const { return state_; }

    /// 周期 tick 所有实例 (推荐 100 Hz)
    static void TickAll() {
        ++current_tick_;
        for (uint8_t i = 0; i < count_; ++i) {
            instances_[i]->Tick();
        }
    }

private:
    void Tick() {
        uint32_t feed = last_feed_tick_;  // 单次读取 volatile

        switch (state_) {
        case State::UNSEEN:
            // 首次喂狗 → 静默上线 (tick 从 1 起, 0 = 未喂过)
            if (feed != 0) {
                state_ = State::ONLINE;
            }
            break;

        case State::ONLINE:
            if (current_tick_ - feed > timeout_ticks_) {
                state_ = State::OFFLINE;
                if (on_offline_) on_offline_(owner_);
            }
            break;

        case State::OFFLINE:
            // 锁存离线, 直到新的 Reload
            if (current_tick_ - feed <= timeout_ticks_) {
                state_ = State::ONLINE;
                if (on_recover_) on_recover_(owner_);
            }
            break;
        }
    }

    uint16_t timeout_ticks_;
    void(*on_offline_)(void*);
    void(*on_recover_)(void*);
    void* owner_;

    volatile uint32_t last_feed_tick_ = 0;  // ISR 写, TickAll 读
    State state_ = State::UNSEEN;           // 仅 TickAll 写

    // 全局时钟 + 实例表 (static inline, C++17 跨 TU 唯一)
    static inline volatile uint32_t current_tick_ = 1;  // 从 1 起, 0 = 未喂过
    static inline DaemonInstance* instances_[MAX_INSTANCES] = {};
    static inline uint8_t count_ = 0;
};

} // namespace daemon
