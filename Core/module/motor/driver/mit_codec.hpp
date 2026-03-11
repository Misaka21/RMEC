#pragma once

#include <cstdint>

/// MIT 协议 float↔uint 编解码工具（DM/HT 电机共用）
namespace mit_codec {

/// 将 float 映射到 N-bit 无符号整数
inline uint16_t FloatToUint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    return static_cast<uint16_t>(
        (x - x_min) * static_cast<float>((1 << bits) - 1) / span);
}

/// 将 N-bit 无符号整数映射回 float
inline float UintToFloat(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span /
           static_cast<float>((1 << bits) - 1) + x_min;
}

/// 打包 MIT 协议 8 字节数据帧
/// 位布局: position[16] | velocity[12] | kp[12] | kd[12] | torque[12] = 64 bits
inline void PackMitFrame(uint8_t* buf,
                         uint16_t pos, uint16_t vel,
                         uint16_t kp, uint16_t kd, uint16_t torque) {
    buf[0] = static_cast<uint8_t>(pos >> 8);
    buf[1] = static_cast<uint8_t>(pos & 0xFF);
    buf[2] = static_cast<uint8_t>(vel >> 4);
    buf[3] = static_cast<uint8_t>(((vel & 0x0F) << 4) | (kp >> 8));
    buf[4] = static_cast<uint8_t>(kp & 0xFF);
    buf[5] = static_cast<uint8_t>(kd >> 4);
    buf[6] = static_cast<uint8_t>(((kd & 0x0F) << 4) | (torque >> 8));
    buf[7] = static_cast<uint8_t>(torque & 0xFF);
}

/// 打包 MIT 模式命令帧: 前 7 字节 0xFF，最后 1 字节为命令码
inline void PackModeFrame(uint8_t* buf, uint8_t cmd) {
    buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = 0xFF; buf[3] = 0xFF;
    buf[4] = 0xFF; buf[5] = 0xFF; buf[6] = 0xFF;
    buf[7] = cmd;
}

} // namespace mit_codec
