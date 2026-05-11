#pragma once

#include "referee_def.hpp"
#include "sal_usart.h"

#include <cstdint>

namespace referee {

// ======================== RefereeParser ========================
/// 裁判系统协议解析器: 字节级状态机 + CRC 校验
///
/// **传输无关**: 不持有 UART, 通过注入的发送函数发送
/// Referee 或 App 层负责创建传输实例并将 rx 回调接到 Parse()
///
/// **接收路径** (ISR 上下文):
///   传输 Rx ISR → Parse(buf, len) → 状态机逐字节 → CRC 校验
///   → 更新 data_ → publish_func_(data_) 直接发布到 Topic
///
/// **发送路径** (Task 上下文):
///   Send(cmd_id, payload, len) → 组帧 + CRC → send_func_()
///   SendInteraction() → 0x0301 交互数据信封
///
/// 协议版本: RoboMaster 2026 V1.2.0
/// 帧格式: [SOF=0xA5][data_length(2B)][seq][CRC8][cmd_id(2B)][data(nB)][CRC16(2B)]

class RefereeParser {
public:
    /// 发送函数签名: 由上层注入, 通常绑定到 UartSend
    using SendFunc = void(*)(uint8_t* buf, uint16_t len);

    /// 带上下文的发送函数签名: 用于模块内绑定多个传输实例
    using SendWithContextFunc = void(*)(void* context, uint8_t* buf, uint16_t len);

    /// 发布回调签名: ISR 上下文, 每解析完一帧调用一次
    /// 典型用法: `[](const RefereeData& d) { referee_topic.Publish(d); }`
    using PublishFunc = void(*)(const RefereeData&);

    /// @param send    发送函数 (Task 上下文调用)
    /// @param publish ISR 回调: 每解析完一帧后调用, 用于直接发布到 Topic
    explicit RefereeParser(SendFunc send = nullptr, PublishFunc publish = nullptr);

    /// @param send    带上下文的发送函数 (Task 上下文调用)
    /// @param context 发送函数上下文
    /// @param publish ISR 回调: 每解析完一帧后调用, 用于直接发布到 Topic
    RefereeParser(SendWithContextFunc send, void* context, PublishFunc publish = nullptr);

    /// ISR 调用: DMA IDLE 收到的原始字节投递到状态机
    /// @note 支持多帧/半帧, 逐字节推进
    void Parse(const uint8_t* buf, uint16_t len);

    /// 编码并发送原始帧 (含帧头 CRC8 + 帧尾 CRC16)
    /// @param cmd_id 命令码 (如 CMD_INTERACTION)
    /// @param payload 数据区内容
    /// @param len 数据区长度
    void Send(uint16_t cmd_id, const uint8_t* payload, uint16_t len);

    /// 发送 0x0301 交互数据, 自动填充 interaction_header (6B)
    /// @param sub_cmd_id  子命令 ID (如 SUB_CMD_DRAW_1, SUB_CMD_SENTRY_CMD)
    /// @param receiver_id 接收者 ID (0 = 自动推算为选手端 = robot_id + 0x0100)
    /// @param content     内容数据段 (不含 interaction_header)
    /// @param content_len 内容长度 (最大 112B)
    void SendInteraction(uint16_t sub_cmd_id, uint16_t receiver_id,
                         const uint8_t* content, uint16_t content_len);

    /// 发送哨兵自主决策指令 (0x0301 + sub_cmd_id 0x0120)
    /// @param sentry_cmd 4B 指令, bit 编码详见 WireSentryCmd
    /// @note receiver_id 固定为 0x8080 (裁判系统服务器)
    void SendSentryDecision(uint32_t sentry_cmd);

    /// 获取本机 robot_id (从最近解析的 0x0201 数据, 单字节原子读)
    /// - 红方: 1=英雄, 2=工程, 3/4/5=步兵, 6=空中, 7=哨兵
    /// - 蓝方: 101=英雄, 102=工程, 103/104/105=步兵, 106=空中, 107=哨兵
    uint8_t RobotId() const { return data_.robot_status.robot_id; }

    /// 推算本机对应的选手端 ID (robot_id + 0x0100)
    /// - 红方: 0x0101~0x0107
    /// - 蓝方: 0x0165~0x016A
    uint16_t ClientId() const {
        return static_cast<uint16_t>(data_.robot_status.robot_id) + 0x0100;
    }

private:
    SendFunc            send_func_             = nullptr;
    SendWithContextFunc send_with_context_func_ = nullptr;
    void*               send_context_          = nullptr;
    PublishFunc         publish_func_          = nullptr;
    RefereeData data_{};

    // 帧解析状态机
    enum class State : uint8_t { WAIT_SOF, HEADER, DATA };
    State    state_       = State::WAIT_SOF;
    uint8_t  rx_buf_[512]{};
    uint16_t rx_idx_      = 0;
    uint16_t data_length_ = 0;
    uint16_t frame_len_   = 0;  ///< 整帧预期长度

    void ProcessFrame();  ///< CRC 全帧校验 → dispatch by cmd_id → 更新 data_ → 发布
};

// ======================== Referee ========================

struct RefereeConfig {
    UART_HandleTypeDef* uart_handle = nullptr;
    uint16_t rx_size = 256;
    sal::UartRxType rx_type = sal::UartRxType::DMA_IDLE;
    sal::UartTxType tx_type = sal::UartTxType::IT;
};

/// 裁判系统 UART 模块: UART RX/TX + RefereeParser.
/// App 层只负责传入硬件 handle、发布回调和离线策略。
class Referee {
public:
    using PublishFunc = RefereeParser::PublishFunc;

    explicit Referee(const RefereeConfig& cfg, PublishFunc publish = nullptr);
    Referee(const Referee&) = delete;
    Referee& operator=(const Referee&) = delete;

    void RestartRx();

    RefereeParser* Parser() { return &parser_; }
    const RefereeParser* Parser() const { return &parser_; }

private:
    static constexpr uint8_t TX_BUFFER_COUNT = 4;
    static constexpr uint16_t TX_BUFFER_SIZE = 512;

    static void SendThunk(void* context, uint8_t* buf, uint16_t len);

    void OnReceive(uint8_t* buf, uint16_t len);
    void Send(uint8_t* buf, uint16_t len);

    RefereeParser parser_;
    sal::UartInstance* uart_ = nullptr;
    uint8_t tx_buf_[TX_BUFFER_COUNT][TX_BUFFER_SIZE] = {};
    uint8_t tx_idx_ = 0;
};

} // namespace referee
