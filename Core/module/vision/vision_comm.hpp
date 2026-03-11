#pragma once

#include "vision_data.hpp"

#include <cstdint>

#define VISION_COMPILER_BARRIER() __asm volatile("" ::: "memory")

namespace vision {

// ======================== VisionComm ========================
/// 视觉通信模块: 协议编解码 + SeqLock
///
/// 传输无关: 不持有 UART/USB 实例, 通过注入的函数指针发送
/// App 层负责创建传输实例并将 rx 回调接到 OnReceive()
///
/// 接收路径 (ISR 上下文):
///   传输 Rx ISR → OnReceive() → CRC 校验 → SeqLock 写 → on_publish_ 回调
///
/// 发送路径 (Task 上下文):
///   VisionTxData → EncodeTxFrame → send_func_()

class VisionComm {
public:
    using SendFunc = void(*)(uint8_t* buf, uint16_t len);
    using PublishCallback = void(*)(const VisionRxData&);

    explicit VisionComm(SendFunc send_func,
                        PublishCallback on_publish = nullptr)
        : send_func_(send_func), on_publish_(on_publish) {}

    /// 编码并发送 (Task 上下文调用)
    void Send(const VisionTxData& data);

    /// 传输层 ISR 调用此函数投递原始帧 (公开, 供 app 层注册为 rx 回调)
    void OnReceive(uint8_t* buf, uint16_t len);

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
    SendFunc send_func_        = nullptr;
    PublishCallback on_publish_ = nullptr;

    VisionRxData rx_data_{};
    volatile uint32_t seq_ = 0;

    uint8_t tx_buf_[32]{};
};

#undef VISION_COMPILER_BARRIER

} // namespace vision
