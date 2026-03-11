#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace daemon {

// ======================== 配置 ========================

struct DaemonConfig {
    uint16_t reload_count = 100;            // 喂狗重载值 (tick 数)
    std::function<void()> callback = nullptr; // 离线回调 (DaemonTask 上下文, 非 ISR)
};

// ======================== DaemonInstance ========================
/// 软件看门狗实例
///
/// 倒计时模型:
///   - ISR / 数据源调用 Reload() "喂狗", 重置计数
///   - DaemonTask 周期调用 TickAll(), 递减所有实例
///   - 计数归零时触发回调 (仅一次), 然后自动重载等待下一轮
///
/// 自注册: 构造时自动加入全局列表, TickAll() 遍历

class DaemonInstance {
public:
    explicit DaemonInstance(const DaemonConfig& cfg)
        : reload_count_(cfg.reload_count),
          temp_count_(cfg.reload_count),
          callback_(cfg.callback)
    {
        Instances().push_back(this);
    }

    /// 喂狗: 重置倒计时 (ISR 安全, 单字写入)
    void Reload() { temp_count_ = reload_count_; }

    /// 在线判断
    bool IsOnline() const { return temp_count_ > 0; }

    /// 周期 tick 所有实例 (推荐 100 Hz)
    static void TickAll() {
        for (auto* d : Instances()) {
            if (d->temp_count_ > 0) {
                if (--d->temp_count_ == 0 && d->callback_) {
                    d->callback_();
                    d->temp_count_ = d->reload_count_; // 自动重载, 周期重试
                }
            }
        }
    }

private:
    uint16_t reload_count_;
    volatile uint16_t temp_count_;
    std::function<void()> callback_;

    /// 延迟初始化的全局列表, 避免 static init order 问题
    static std::vector<DaemonInstance*>& Instances() {
        static std::vector<DaemonInstance*> v;
        return v;
    }
};

} // namespace daemon
