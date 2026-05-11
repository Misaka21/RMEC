#include "vision_comm.hpp"
#include "vision_protocol.hpp"

#define VISION_COMPILER_BARRIER() __asm volatile("" ::: "memory")

namespace vision {

VisionComm::VisionComm(const VisionConfig& cfg, PublishCallback on_publish)
    : transport_(cfg.transport), on_publish_(on_publish) {
    if (transport_ == VisionTransport::USB_VCP) {
        sal::UsbInstance::UsbConfig usb_cfg{};
        usb_cfg.rx_cbk = [this](uint8_t* buf, uint16_t len) {
            OnReceive(buf, len);
        };
        usb_ = new sal::UsbInstance(usb_cfg);
        return;
    }

    sal::UartInstance::UartConfig uart_cfg{};
    uart_cfg.handle  = cfg.uart_handle;
    uart_cfg.rx_size = FRAME_SIZE;
    uart_cfg.rx_type = cfg.uart_rx_type;
    uart_cfg.rx_cbk  = [this](uint8_t* buf, uint16_t len) {
        OnReceive(buf, len);
    };
    uart_cfg.tx_type = cfg.uart_tx_type;

    uart_ = new sal::UartInstance(uart_cfg);
    RestartRx();
}

void VisionComm::Send(const VisionTxData& data) {
    static_assert(FRAME_SIZE <= TX_BUFFER_SIZE);

    auto* tx_buf = tx_buf_[tx_idx_];
    tx_idx_ = static_cast<uint8_t>((tx_idx_ + 1) % TX_BUFFER_COUNT);

    EncodeTxFrame(data, tx_buf);
    SendRaw(tx_buf, FRAME_SIZE);
}

void VisionComm::RestartRx() {
    if (uart_) uart_->UartRestartRecv();
}

void VisionComm::OnReceive(uint8_t* buf, uint16_t len) {
    if (len != FRAME_SIZE) return;

    VisionRxData curr{};
    if (!DecodeRxFrame(buf, curr)) return;

    ++seq_;
    VISION_COMPILER_BARRIER();
    rx_data_ = curr;
    VISION_COMPILER_BARRIER();
    ++seq_;

    if (on_publish_) on_publish_(curr);
}

void VisionComm::SendRaw(uint8_t* buf, uint16_t len) {
    if (transport_ == VisionTransport::USB_VCP) {
        if (usb_) usb_->Transmit(buf, len);
        return;
    }

    if (uart_) uart_->UartSend(buf, len);
}

#undef VISION_COMPILER_BARRIER

} // namespace vision
