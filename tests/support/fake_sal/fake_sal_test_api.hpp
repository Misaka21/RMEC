#pragma once
// fake_sal 测试侧 API: 断言发送、注入接收、控制时间 (设计文档 5.2 节)
#include <cstdint>
#include <vector>

struct UART_HandleTypeDef;
struct CAN_HandleTypeDef;

namespace test {

struct UartPacket {
    UART_HandleTypeDef* handle;
    std::vector<uint8_t> data;
};
std::vector<UartPacket>& SentUartPackets();
void InjectUartRx(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len);

struct CanFrame {
    CAN_HandleTypeDef* handle;
    uint32_t std_id;
    uint8_t data[8];
};
std::vector<CanFrame>& SentCanFrames();
void InjectCanRx(CAN_HandleTypeDef* hcan, uint32_t std_id, const uint8_t (&data)[8]);

// 推进假 DWT 时钟 (按 DwtInit(168) 的 168 MHz 折算)
void AdvanceTimeMs(uint32_t ms);

// 清空收发记录 (不清实例注册表; SAL 实例生命周期为全局)
void ResetFakeRecords();

}  // namespace test
