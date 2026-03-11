#include "vision_comm.hpp"
#include "vision_protocol.hpp"

#define VISION_COMPILER_BARRIER() __asm volatile("" ::: "memory")

namespace vision {

void VisionComm::Send(const VisionTxData& data) {
    EncodeTxFrame(data, tx_buf_);
    if (send_func_) send_func_(tx_buf_, FRAME_SIZE);
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

#undef VISION_COMPILER_BARRIER

} // namespace vision
