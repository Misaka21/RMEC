// 链接期替换 sal_can.cc 的假实现: 发送进记录表, 接收由测试注入
// 成员函数在此定义, 因而拥有私有成员访问权, 借此把注入通道登记到测试注册表
#include "sal_can.h"

#include "fake_sal_test_api.hpp"

#include <cstring>
#include <functional>
#include <vector>

namespace {

struct FakeCanEntry {
    CAN_HandleTypeDef* handle;
    uint32_t rx_id;
    std::function<void(const uint8_t*, uint8_t)> inject;
};

std::vector<FakeCanEntry>& Registry() {
    static std::vector<FakeCanEntry> r;
    return r;
}

}  // namespace

namespace test {

void InjectCanRx(CAN_HandleTypeDef* hcan, uint32_t std_id, const uint8_t (&data)[8]) {
    for (auto& e : Registry())
        if (e.handle == hcan && e.rx_id == std_id) e.inject(data, 8);
}

}  // namespace test

namespace sal {

std::vector<CanInstance::CanPtr> CanInstance::instance_[CAN_DEV_NUM]{};
CanInstance::FifoPtr CanInstance::fifo_ptr_[CAN_DEV_NUM]{};
uint8_t CanInstance::can1_filter_idx_ = 0;
uint8_t CanInstance::can2_filter_idx_ = 0;

CanInstance::CanInstance(const CanConfig& config)
    : handle_(config.handle),
      tx_header_{},
      tx_mailbox_(0),
      tx_id_(config.tx_id),
      tx_len_(8),
      tx_cbk_(config.tx_cbk),
      fifo_timeout_us_(config.fifo_timeout_us),
      rx_id_(config.rx_id),
      rx_data_{},
      rx_cbk_(config.rx_cbk),
      busy_cnt_(0),
      wait_time_(0) {
    Registry().push_back({handle_, rx_id_, [this](const uint8_t* data, uint8_t len) {
        std::memcpy(rx_data_, data, len);
        if (rx_cbk_) rx_cbk_(len);
    }});
}

void CanInstance::CanServiceInit() {}
void CanInstance::CanAddFilter() {}

bool CanInstance::CanTransmit(const CanMsg& msg, uint16_t) {
    test::CanFrame f{handle_, tx_id_, {}};
    std::memcpy(f.data, msg.data, 8);
    test::SentCanFrames().push_back(f);
    if (tx_cbk_) tx_cbk_();
    return true;
}

void CanInstance::CanFifoxCallback(CAN_HandleTypeDef*, uint32_t) {}
void CanInstance::CanTxFinishCallback(CAN_HandleTypeDef*) {}
void CanInstance::TxQueueFront(FifoPtr&) {}
void CanInstance::PopQueueCallback(FifoPtr&) {}

}  // namespace sal
