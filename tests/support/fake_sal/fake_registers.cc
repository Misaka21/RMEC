// 假外设句柄/假寄存器的唯一定义 + 跨 fake 模块的公共测试工具
#include "main.h"
#include "usart.h"

#include "fake_sal_test_api.hpp"

extern "C" {
FakeDwtType       fake_dwt{};
FakeCoreDebugType fake_core_debug{};

UART_HandleTypeDef huart1{1};
UART_HandleTypeDef huart3{3};
UART_HandleTypeDef huart6{6};
}

namespace test {

std::vector<UartPacket>& SentUartPackets() {
    static std::vector<UartPacket> v;
    return v;
}

std::vector<CanFrame>& SentCanFrames() {
    static std::vector<CanFrame> v;
    return v;
}

void AdvanceTimeMs(uint32_t ms) { fake_dwt.CYCCNT += ms * 168000u; }

void ResetFakeRecords() {
    SentUartPackets().clear();
    SentCanFrames().clear();
}

}  // namespace test
