#pragma once

#include "sal_can.h"
#include "daemon.hpp"

#include <cstdint>
#include <cstring>
#include <type_traits>

struct CanCommConfig {
    CAN_HandleTypeDef* can_handle;
    uint16_t base_tx_id;         // 发送基础 ID (连续分配 kTxFrames 个)
    uint16_t base_rx_id;         // 接收基础 ID (连续分配 kRxFrames 个)
    uint16_t daemon_timeout = 0; // 0 = 不创建 Daemon
};

template <typename TxData, typename RxData>
class CanComm {
    static_assert(std::is_trivially_copyable_v<TxData>);
    static_assert(std::is_trivially_copyable_v<RxData>);

public:
    explicit CanComm(const CanCommConfig& cfg);

    /// 切片发送 N 帧 (task context 调用)
    void Send(const TxData& data);

    /// 返回最新接收数据引用 (ISR 异步更新)
    const RxData& Recv() const { return rx_buf_; }

    /// Daemon 在线判断
    bool IsOnline() const { return daemon_ && daemon_->IsOnline(); }

private:
    static constexpr int kTxFrames = (sizeof(TxData) + 7) / 8;
    static constexpr int kRxFrames = (sizeof(RxData) + 7) / 8;
    static constexpr int kInstances = kTxFrames > kRxFrames ? kTxFrames : kRxFrames;

    sal::CanInstance* instances_[kInstances] = {};
    RxData rx_buf_{};
    daemon::DaemonInstance* daemon_ = nullptr;
};

// ======================== 实现 ========================

template <typename TxData, typename RxData>
CanComm<TxData, RxData>::CanComm(const CanCommConfig& cfg) {
    if (cfg.daemon_timeout > 0) {
        daemon_ = new daemon::DaemonInstance({.timeout_ticks = cfg.daemon_timeout});
    }

    for (int i = 0; i < kInstances; ++i) {
        sal::CanInstance::CanConfig can_cfg{};
        can_cfg.handle = cfg.can_handle;
        can_cfg.tx_id  = cfg.base_tx_id + i;
        can_cfg.rx_id  = cfg.base_rx_id + i;

        if (i < kRxFrames) {
            can_cfg.rx_cbk = [this, i](uint8_t /*len*/) {
                uint8_t* dst = reinterpret_cast<uint8_t*>(&rx_buf_) + i * 8;
                uint8_t n = (i == kRxFrames - 1)
                    ? static_cast<uint8_t>(sizeof(RxData) - i * 8) : 8;
                std::memcpy(dst, instances_[i]->RxData(), n);
                if (daemon_) daemon_->Reload();
            };
        }

        instances_[i] = new sal::CanInstance(can_cfg);
    }
}

template <typename TxData, typename RxData>
void CanComm<TxData, RxData>::Send(const TxData& data) {
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&data);
    for (int i = 0; i < kTxFrames; ++i) {
        sal::CanMsg msg{};
        uint8_t n = (i == kTxFrames - 1)
            ? static_cast<uint8_t>(sizeof(TxData) - i * 8) : 8;
        std::memcpy(msg.data, raw + i * 8, n);
        instances_[i]->CanTransmit(msg);
    }
}
