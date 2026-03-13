#pragma once

#include "referee_def.hpp"

#include <cstdint>

#define REFEREE_COMPILER_BARRIER() __asm volatile("" ::: "memory")

namespace referee {

// ======================== RefereeParser ========================
/// 裁判系统协议解析器: 字节级状态机 + CRC 校验 + SeqLock
///
/// 传输无关: 不持有 UART, 通过注入的函数指针发送
/// App 层负责创建传输实例并将 rx 回调接到 Parse()
///
/// 接收路径 (ISR 上下文):
///   传输 Rx ISR → Parse(buf, len) → 状态机逐字节 → CRC 校验
///   → SeqLock 写 data_
///
/// 读取路径 (Task 上下文):
///   Read(out) → SeqLock 一致性快照读, 返回 true 表示成功
///
/// 发送路径 (Task 上下文):
///   Send(cmd_id, payload, len) → 组帧 + CRC → send_func_()

class RefereeParser {
public:
    using SendFunc = void(*)(uint8_t* buf, uint16_t len);

    explicit RefereeParser(SendFunc send = nullptr);

    /// ISR 调用: DMA IDLE 收到的原始字节投递到状态机
    void Parse(const uint8_t* buf, uint16_t len);

    /// Task 上下文: SeqLock 一致性快照读, 返回 true 表示读到完整数据
    bool Read(RefereeData& out) const {
        uint32_t s1 = seq_;
        REFEREE_COMPILER_BARRIER();
        if (s1 & 1u) return false;  // ISR 正在写
        out = data_;
        REFEREE_COMPILER_BARRIER();
        return s1 == seq_;
    }

    /// 编码并发送原始帧 (含帧头 CRC8 + 帧尾 CRC16)
    void Send(uint16_t cmd_id, const uint8_t* payload, uint16_t len);

    /// 发送 0x0301 交互数据, 自动填充 interaction_header
    /// @param sub_cmd_id  子命令 ID (如 SUB_CMD_DRAW_1, SUB_CMD_SENTRY_CMD)
    /// @param receiver_id 接收者 ID (0 = 自动推算选手端 = robot_id + 0x0100)
    /// @param content     内容数据段 (不含 interaction_header)
    /// @param content_len 内容长度
    void SendInteraction(uint16_t sub_cmd_id, uint16_t receiver_id,
                         const uint8_t* content, uint16_t content_len);

    /// 发送哨兵自主决策指令 (0x0301 + sub_cmd_id 0x0120)
    void SendSentryDecision(uint32_t sentry_cmd);

    /// 获取本机 robot_id (从最近解析的 0x0201 数据, 单字节原子读)
    uint8_t RobotId() const { return data_.robot_status.robot_id; }

    /// 推算本机对应的选手端 ID (robot_id + 0x0100)
    uint16_t ClientId() const {
        return static_cast<uint16_t>(data_.robot_status.robot_id) + 0x0100;
    }

private:
    SendFunc send_func_ = nullptr;
    RefereeData data_{};
    volatile uint32_t seq_ = 0;  // SeqLock 序列号

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

#undef REFEREE_COMPILER_BARRIER
