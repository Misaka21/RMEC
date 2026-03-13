#pragma once

#include "referee_def.hpp"

#include <cstdint>

namespace referee {

// ======================== RefereeParser ========================
/// 裁判系统协议解析器: 字节级状态机 + CRC 校验
///
/// 传输无关: 不持有 UART, 通过注入的函数指针发送
/// App 层负责创建传输实例并将 rx 回调接到 Parse()
///
/// 接收路径 (ISR 上下文):
///   传输 Rx ISR → Parse(buf, len) → 状态机逐字节 → CRC 校验 → 更新 data_
///
/// 发送路径 (Task 上下文):
///   Send(cmd_id, payload, len) → 组帧 + CRC → send_func_()

class RefereeParser {
public:
    using SendFunc = void(*)(uint8_t* buf, uint16_t len);

    explicit RefereeParser(SendFunc send = nullptr);

    /// ISR 调用: DMA IDLE 收到的原始字节投递到状态机
    void Parse(const uint8_t* buf, uint16_t len);

    /// Task 上下文: 读取最新解析结果
    const RefereeData& Data() const { return data_; }

    /// 编码并发送交互数据 (含帧头 CRC8 + 帧尾 CRC16)
    void Send(uint16_t cmd_id, const uint8_t* payload, uint16_t len);

private:
    SendFunc send_func_ = nullptr;
    RefereeData data_{};

    // 帧解析状态机
    enum class State : uint8_t { WAIT_SOF, HEADER, DATA };
    State    state_       = State::WAIT_SOF;
    uint8_t  rx_buf_[512]{};
    uint16_t rx_idx_      = 0;
    uint16_t data_length_ = 0;
    uint16_t frame_len_   = 0;  // 整帧预期长度

    void ProcessFrame();  // CRC 全帧校验 → dispatch by cmd_id → 更新 data_
};

} // namespace referee
