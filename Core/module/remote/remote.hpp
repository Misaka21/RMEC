#pragma once

#include <cstdint>
#include <type_traits>
#include "sal_usart.h"

namespace remote {

// Cortex-M4 单核无乱序执行, 仅需阻止编译器重排
#define REMOTE_COMPILER_BARRIER() __asm volatile("" ::: "memory")

// ======================== 配置 ========================

struct RemoteConfig {
    UART_HandleTypeDef* uart_handle = nullptr;
    uint16_t offline_threshold = 100;   // app bridge 离线判定阈值（周期数）
};

// ======================== 编译期协议契约检查 ========================

template <typename P>
constexpr bool CheckProtocolInterface() {
    static_assert(P::FRAME_SIZE > 0,
        "Protocol::FRAME_SIZE must be positive");
    static_assert(std::is_trivially_copyable_v<typename P::Data>,
        "Protocol::Data must be trivially copyable");
    static_assert(std::is_same_v<
        decltype(&P::Decode),
        void(*)(const uint8_t*, typename P::Data&, const typename P::Data&)>,
        "Protocol must have: static void Decode(const uint8_t*, Data&, const Data&)");
    static_assert(std::is_same_v<
        decltype(&P::Reset),
        void(*)(typename P::Data&)>,
        "Protocol must have: static void Reset(Data&)");
    return true;
}

// ======================== Remote<Protocol> ========================
/// 遥控器模板类, Protocol 为编译期策略 (零虚函数)
///
/// UART ISR 驱动的数据源:
///   ISR → Decode → 更新内部快照
///
/// App 层只需:
///   1. 初始化 Remote 实例
///   2. 在 bridge/pump 中调用 ReadSnapshot()
///   3. 按需发布 Topic（单写者由 app 保证）

template <typename Protocol>
class Remote {
    static_assert(CheckProtocolInterface<Protocol>());

public:
    using Data = typename Protocol::Data;
    using PublishCallback = void(*)(const Data&);

    explicit Remote(const RemoteConfig& cfg, PublishCallback on_publish = nullptr) {
        on_publish_ = on_publish;
        sal::UartInstance::UartConfig uart_cfg{};
        uart_cfg.handle  = cfg.uart_handle;
        uart_cfg.rx_size = Protocol::FRAME_SIZE;
        uart_cfg.rx_type = sal::UartRxType::DMA_IDLE;
        uart_cfg.rx_cbk  = [this](uint8_t* buf, uint16_t len) {
            OnReceive(buf, len);
        };

        uart_ = new sal::UartInstance(uart_cfg);
        uart_->UartRestartRecv();
    }

    /// 读取一致快照（无锁，可能因并发写返回 false）
    bool ReadSnapshot(Data& out, uint32_t* seq_out = nullptr) const {
        uint32_t s1 = seq_;
        REMOTE_COMPILER_BARRIER();
        if (s1 & 1u) return false;  // ISR 正在写
        out = data_;
        REMOTE_COMPILER_BARRIER();
        uint32_t s2 = seq_;
        if (s1 != s2) return false; // 读取被打断
        if (seq_out) *seq_out = s1;
        return true;
    }

    /// 最近一次成功写入的序号（偶数）
    uint32_t SnapshotSeq() const { return seq_; }

    /// 离线恢复: Reset 数据 → seqlock 写入 → 回调发布 → 重启 UART
    /// 由 app 离线检测调用, 保证 Publish 只经由 on_publish_ 一条路径
    void ResetAndPublish() {
        Data zero{};
        Protocol::Reset(zero);
        last_data_ = zero;

        ++seq_;
        REMOTE_COMPILER_BARRIER();
        data_ = zero;
        REMOTE_COMPILER_BARRIER();
        ++seq_;

        if (on_publish_) on_publish_(zero);
        if (uart_) uart_->UartRestartRecv();
    }

private:
    /// UART 接收回调 (ISR 上下文)
    void OnReceive(uint8_t* buf, uint16_t len) {
        if (len != Protocol::FRAME_SIZE) return;
        Data curr{};
        Protocol::Decode(buf, curr, last_data_);
        last_data_ = curr;

        ++seq_;                 // odd: writing
        REMOTE_COMPILER_BARRIER();
        data_ = curr;
        REMOTE_COMPILER_BARRIER();
        ++seq_;                 // even: write done

        if (on_publish_) on_publish_(curr);
    }

    sal::UartInstance* uart_;
    PublishCallback on_publish_ = nullptr;
    Data data_{};
    Data last_data_{};
    volatile uint32_t seq_ = 0;
};

#undef REMOTE_COMPILER_BARRIER

} // namespace remote
