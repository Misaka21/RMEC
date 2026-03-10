#pragma once

#include <cstdint>
#include <type_traits>
#include "sal_usart.h"

namespace remote {

// ======================== 配置 ========================

struct RemoteConfig {
    UART_HandleTypeDef* uart_handle = nullptr;
    uint16_t offline_threshold = 100;   // Tick() 周期数, 超过视为离线
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
///   ISR → Decode → publish 回调 (由 app 层注入, 发布到 Topic)
///
/// App 层只需:
///   1. 构造时注入 publish 回调
///   2. 定期调用 Tick() 做离线检测
///   3. 消费者订阅 Topic 即可

template <typename Protocol>
class Remote {
    static_assert(CheckProtocolInterface<Protocol>());

public:
    using Data = typename Protocol::Data;
    using PublishFn = void(*)(const Data&);

    Remote(const RemoteConfig& cfg, PublishFn publish)
        : publish_(publish),
          offline_cnt_(cfg.offline_threshold),
          offline_threshold_(cfg.offline_threshold)
    {
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

    /// 在线判断
    bool IsOnline() const { return offline_cnt_ < offline_threshold_; }

    /// 定期调用: 递增离线计数, 超时后清零数据并重启 UART
    void Tick() {
        if (offline_cnt_ < offline_threshold_) {
            offline_cnt_++;
        } else if (!was_offline_) {
            Protocol::Reset(data_);
            last_data_ = data_;
            if (publish_) publish_(data_);   // 发布清零后的数据
            uart_->UartRestartRecv();
            was_offline_ = true;
        }
    }

private:
    /// UART 接收回调 (ISR 上下文)
    void OnReceive(uint8_t* buf, uint16_t len) {
        if (len != Protocol::FRAME_SIZE) return;
        Protocol::Decode(buf, data_, last_data_);
        last_data_ = data_;
        if (publish_) publish_(data_);       // ISR 中直接发布
        offline_cnt_ = 0;
        was_offline_ = false;
    }

    PublishFn publish_;
    sal::UartInstance* uart_;
    Data data_{};
    Data last_data_{};
    uint16_t offline_cnt_;
    uint16_t offline_threshold_;
    bool was_offline_ = true;
};

} // namespace remote
