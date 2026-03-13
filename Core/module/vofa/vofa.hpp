#pragma once

#include "sal_usart.h"
#include <cstdint>
#include <cstring>

namespace vofa {

inline constexpr uint8_t JUSTFLOAT_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

struct VofaConfig {
    UART_HandleTypeDef* uart_handle = nullptr;
};

/// VOFA+ JustFloat 绘图模块
///
/// 模式 A: 模块内创建 SAL, 调用者只传 HAL handle
/// 协议: [float0][float1]...[floatN-1][0x00 0x00 0x80 0x7F]
///
/// @tparam N 通道数 (编译期确定, 零动态分配)
///
/// SetChannel() 可从任意 task 调用 (单 float 写入在 Cortex-M4 上原子)
/// Send() 由 vofa_task 固定频率调用

template <uint8_t N>
class Vofa {
    static_assert(N > 0 && N <= 32, "channel count must be 1..32");

public:
    explicit Vofa(const VofaConfig& config) {
        sal::UartInstance::UartConfig uart_cfg{};
        uart_cfg.handle  = config.uart_handle;
        uart_cfg.tx_type = sal::UartTxType::DMA;
        uart_ = new sal::UartInstance(uart_cfg);
    }

    void SetChannel(uint8_t ch, float val) {
        if (ch < N) channels_[ch] = val;
    }

    void Send() {
        std::memcpy(buf_, channels_, sizeof(float) * N);
        std::memcpy(buf_ + sizeof(float) * N, JUSTFLOAT_TAIL, 4);
        uart_->UartSend(buf_, sizeof(buf_));
    }

private:
    sal::UartInstance* uart_ = nullptr;
    float channels_[N] = {};
    uint8_t buf_[sizeof(float) * N + 4] = {};
};

} // namespace vofa
