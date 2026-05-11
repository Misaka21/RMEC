#pragma once

#include "sal_usb.h"
#include "sal_usart.h"
#include "vision_data.hpp"

#include <cstdint>

#define VISION_COMPILER_BARRIER() __asm volatile("" ::: "memory")

namespace vision {

enum class VisionTransport : uint8_t {
    USB_VCP,
    UART,
};

struct VisionConfig {
    VisionTransport transport = VisionTransport::USB_VCP;
    UART_HandleTypeDef* uart_handle = nullptr;
    sal::UartRxType uart_rx_type = sal::UartRxType::DMA_IDLE;
    sal::UartTxType uart_tx_type = sal::UartTxType::IT;
};

// ======================== VisionComm ========================
/// 视觉通信模块: USB/UART 传输 + 协议编解码 + SeqLock
///
/// App 层只负责传入硬件配置、发布回调和离线策略。
///
/// 接收路径 (ISR 上下文):
///   传输 Rx ISR → OnReceive() → CRC 校验 → SeqLock 写 → on_publish_ 回调
///
/// 发送路径 (Task 上下文):
///   VisionTxData → EncodeTxFrame → SendRaw()

class VisionComm {
public:
    using PublishCallback = void(*)(const VisionRxData&);

    explicit VisionComm(const VisionConfig& cfg,
                        PublishCallback on_publish = nullptr);
    VisionComm(const VisionComm&) = delete;
    VisionComm& operator=(const VisionComm&) = delete;

    /// 编码并发送 (Task 上下文调用)
    void Send(const VisionTxData& data);

    /// 重启接收 (UART 有效, USB VCP 为 no-op)
    void RestartRx();

    /// SeqLock 读取最新接收数据, 返回 true 表示读到一致快照
    bool Recv(VisionRxData& out) const {
        uint32_t s1 = seq_;
        VISION_COMPILER_BARRIER();
        if (s1 & 1u) return false;
        out = rx_data_;
        VISION_COMPILER_BARRIER();
        return s1 == seq_;
    }

private:
    static constexpr uint8_t TX_BUFFER_COUNT = 4;
    static constexpr uint16_t TX_BUFFER_SIZE = 32;

    void OnReceive(uint8_t* buf, uint16_t len);
    void SendRaw(uint8_t* buf, uint16_t len);

    VisionTransport transport_ = VisionTransport::USB_VCP;
    sal::UsbInstance* usb_ = nullptr;
    sal::UartInstance* uart_ = nullptr;
    PublishCallback on_publish_ = nullptr;

    VisionRxData rx_data_{};
    volatile uint32_t seq_ = 0;

    uint8_t tx_buf_[TX_BUFFER_COUNT][TX_BUFFER_SIZE] = {};
    uint8_t tx_idx_ = 0;
};

#undef VISION_COMPILER_BARRIER

} // namespace vision
