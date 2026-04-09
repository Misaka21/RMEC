#pragma once

#include <cstdint>

namespace sal {

// ============================================================
// SAL 层基础错误码
// ============================================================

enum class Error : uint8_t {
    OK = 0,                    // 成功
    TIMEOUT,                   // 超时
    BUSY,                      // 资源忙
    INVALID_STATE,             // 状态错误
    HARDWARE_NOT_FOUND,        // 设备未找到
    HARDWARE_COMM_FAIL,        // 通信失败
    HARDWARE_CRC_FAIL,         // CRC 校验失败
    HARDWARE_TIMEOUT,          // 硬件响应超时
    NO_MEMORY,                 // 内存不足
    INVALID_CONFIG,            // 配置参数错误
    OUT_OF_RANGE,              // 参数越界
    NOT_IMPLEMENTED,           // 功能未实现
};

// 错误码转字符串
inline const char* ErrorToString(Error e) {
    switch (e) {
        case Error::OK:                    return "OK";
        case Error::TIMEOUT:               return "TIMEOUT";
        case Error::BUSY:                  return "BUSY";
        case Error::INVALID_STATE:         return "INVALID_STATE";
        case Error::HARDWARE_NOT_FOUND:    return "HARDWARE_NOT_FOUND";
        case Error::HARDWARE_COMM_FAIL:    return "HARDWARE_COMM_FAIL";
        case Error::HARDWARE_CRC_FAIL:     return "HARDWARE_CRC_FAIL";
        case Error::HARDWARE_TIMEOUT:      return "HARDWARE_TIMEOUT";
        case Error::NO_MEMORY:             return "NO_MEMORY";
        case Error::INVALID_CONFIG:        return "INVALID_CONFIG";
        case Error::OUT_OF_RANGE:          return "OUT_OF_RANGE";
        case Error::NOT_IMPLEMENTED:       return "NOT_IMPLEMENTED";
        default:                           return "UNKNOWN_ERROR";
    }
}

} // namespace sal
