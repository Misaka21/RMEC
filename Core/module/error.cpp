#include "error.hpp"
#include <cstring>

namespace error {

// ============================================================
// 错误码字符串
// ============================================================

static const char* error_strings[] = {
    "OK",
    "NO_MEMORY",
    "TIMEOUT",
    "BUSY",
    "INVALID_STATE",
    "HARDWARE_NOT_FOUND",
    "HARDWARE_COMM_FAIL",
    "HARDWARE_CRC_FAIL",
    "HARDWARE_TIMEOUT",
    "INVALID_CONFIG",
    "NULL_POINTER",
    "OUT_OF_RANGE",
    "NOT_IMPLEMENTED",
    "STACK_OVERFLOW",
    "BUFFER_OVERFLOW",
    "MATH_ERROR",
};

const char* ErrorToString(Error e) {
    uint8_t idx = static_cast<uint8_t>(e);
    if (idx < sizeof(error_strings) / sizeof(error_strings[0])) {
        return error_strings[idx];
    }
    if (idx >= 64) return "USER_ERROR";
    return "UNKNOWN_ERROR";
}

// ============================================================
// 断言失败处理
// ============================================================

static volatile struct {
    const char* file;
    int line;
    const char* msg;
} assert_info_;

void ReportAssertionFailure(const char* file, int line, const char* msg) {
    assert_info_.file = file;
    assert_info_.line = line;
    assert_info_.msg = msg;

    RecordError(Error::INVALID_STATE);
}

// ============================================================
// 错误统计
// ============================================================

static ErrorStats stats_;

void RecordError(Error e) {
    stats_.total_errors++;
    stats_.error_counts[static_cast<uint8_t>(e) & 0x0F]++;
    stats_.last_error = e;
    // 注意：时间戳由调用方填充（通过 HAL_GetTick() 或 osKernelSysTick()）
}

const ErrorStats& GetErrorStats() {
    return stats_;
}

void ResetErrorStats() {
    stats_ = ErrorStats{};
}

// ============================================================
// ErrorHandler 实现
// ============================================================

ErrorHandler::ErrorHandler(const char* name, const ErrorHandlerConfig& cfg)
    : name_(name), cfg_(cfg) {}

bool ErrorHandler::Handle(Error e, const char* context) {
    if (e == Error::OK) {
        if (consecutive_errors_ > 0 && cfg_.on_recovery) {
            cfg_.on_recovery(Error::OK, name_);
        }
        consecutive_errors_ = 0;
        return true;
    }

    RecordError(e);
    consecutive_errors_++;

    // 调用错误回调
    if (cfg_.on_error) {
        cfg_.on_error(e, context ? context : name_);
    }

    switch (cfg_.strategy) {
    case RecoveryStrategy::IGNORE:
        return true;

    case RecoveryStrategy::RETRY:
        if (consecutive_errors_ <= cfg_.max_retries) {
            // 指数退避
            uint32_t delay = cfg_.retry_delay_ms * (1u << (consecutive_errors_ - 1));
            last_retry_delay_ms_ = delay;
            if (cfg_.delay_hook) {
                cfg_.delay_hook(delay);
            }
            return true;  // 调用者应重试
        }
        return false;  // 重试次数用尽

    case RecoveryStrategy::DEGRADE:
        return true;

    case RecoveryStrategy::HALT:
        return false;

    case RecoveryStrategy::REBOOT:
        if (cfg_.reboot_hook)
            cfg_.reboot_hook();
        return false;  // hook 为 null 时退化为 HALT
    }

    return false;
}

void ErrorHandler::Reset() {
    consecutive_errors_ = 0;
    last_retry_delay_ms_ = 0;
}

} // namespace error
