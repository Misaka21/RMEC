// 链接期替换 sal_usart.cc 的假实现: 发送进记录表, 接收由测试注入
// 成员函数在此定义, 因而拥有私有成员访问权, 借此把注入通道登记到测试注册表
#include "sal_usart.h"

#include "fake_sal_test_api.hpp"

#include <cstring>
#include <functional>
#include <vector>

namespace {

struct FakeUartEntry {
    UART_HandleTypeDef* handle;
    std::function<void(const uint8_t*, uint16_t)> inject;
};

std::vector<FakeUartEntry>& Registry() {
    static std::vector<FakeUartEntry> r;
    return r;
}

}  // namespace

namespace test {

void InjectUartRx(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len) {
    for (auto& e : Registry())
        if (e.handle == huart) e.inject(data, len);
}

}  // namespace test

namespace sal {

std::vector<UartInstance::UartPtr> UartInstance::instance_list_{};

UartInstance::UartInstance(const UartConfig& config)
    : handle_(config.handle),
      rx_size_(config.rx_size),
      rx_buff_{},
      rx_type_(config.rx_type),
      rx_cbk_(config.rx_cbk),
      tx_use_fifo_(config.use_fifo),
      tx_queue_mx_size_(config.queue_mx_size),
      tx_type_(config.tx_type),
      tx_cbk_(config.tx_cbk) {
    instance_list_.push_back(this);
    Registry().push_back({handle_, [this](const uint8_t* data, uint16_t len) {
        uint16_t n = len < UART_MX_RX_BUFFER_SIZE ? len : UART_MX_RX_BUFFER_SIZE;
        std::memcpy(rx_buff_, data, n);
        if (rx_cbk_) rx_cbk_(rx_buff_, n);
    }});
}

UartTxState UartInstance::UartSend(uint8_t* data, uint16_t size, uint32_t) {
    test::SentUartPackets().push_back({handle_, {data, data + size}});
    if (tx_cbk_) tx_cbk_();
    return UartTxState::BLOCK_FINISH;
}

void UartInstance::UartSetSendType(UartTxType type, bool use_fifo, uint8_t queue_mx_size) {
    tx_type_ = type;
    tx_use_fifo_ = use_fifo;
    tx_queue_mx_size_ = queue_mx_size;
}

uint16_t UartInstance::UartRecv(uint8_t*, uint16_t, uint32_t) { return 0; }

void UartInstance::UartSetRecvType(UartRxType type, uint16_t size) {
    rx_type_ = type;
    rx_size_ = size;
}

void UartInstance::UartRestartRecv() {}
void UartInstance::UartStopRecv() {}
void UartInstance::PopSend() {}

}  // namespace sal
