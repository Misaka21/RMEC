#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <utility>

// SAL 层错误码（用于转换）
#include "sal/error.hpp"

// ============================================================
// 模块层扩展错误码
// ============================================================

namespace error {

enum class Error : uint8_t {
    OK = 0,                    // 成功

    // 系统级错误 (1-15)
    NO_MEMORY,                 // 内存不足
    TIMEOUT,                   // 超时
    BUSY,                      // 资源忙
    INVALID_STATE,             // 状态错误

    // 硬件级错误 (16-31)
    HARDWARE_NOT_FOUND,        // 设备未找到
    HARDWARE_COMM_FAIL,        // 通信失败
    HARDWARE_CRC_FAIL,         // CRC 校验失败
    HARDWARE_TIMEOUT,          // 硬件响应超时

    // 配置级错误 (32-47)
    INVALID_CONFIG,            // 配置参数错误
    NULL_POINTER,              // 空指针
    OUT_OF_RANGE,              // 参数越界
    NOT_IMPLEMENTED,           // 功能未实现

    // 运行时错误 (48-63)
    STACK_OVERFLOW,            // 栈溢出
    BUFFER_OVERFLOW,           // 缓冲区溢出
    MATH_ERROR,                // 数学错误

    // 用户自定义错误起始点
    USER_ERROR = 64,
};

// 从 SAL 错误码转换
inline Error FromSalError(sal::Error e) {
    switch (e) {
        case sal::Error::OK:                 return Error::OK;
        case sal::Error::TIMEOUT:            return Error::TIMEOUT;
        case sal::Error::BUSY:               return Error::BUSY;
        case sal::Error::INVALID_STATE:      return Error::INVALID_STATE;
        case sal::Error::HARDWARE_NOT_FOUND: return Error::HARDWARE_NOT_FOUND;
        case sal::Error::HARDWARE_COMM_FAIL: return Error::HARDWARE_COMM_FAIL;
        case sal::Error::HARDWARE_CRC_FAIL:  return Error::HARDWARE_CRC_FAIL;
        case sal::Error::HARDWARE_TIMEOUT:   return Error::HARDWARE_TIMEOUT;
        case sal::Error::NO_MEMORY:          return Error::NO_MEMORY;
        case sal::Error::INVALID_CONFIG:     return Error::INVALID_CONFIG;
        case sal::Error::OUT_OF_RANGE:       return Error::OUT_OF_RANGE;
        case sal::Error::NOT_IMPLEMENTED:    return Error::NOT_IMPLEMENTED;
        default:                             return Error::INVALID_STATE;
    }
}

// ============================================================
// Result<T> - 轻量级结果类型
// ============================================================

template<typename T>
class Result {
    static_assert(std::is_trivially_destructible_v<T>,
                  "Result<T>: T must be trivially destructible");

public:
    using ValueType = T;

    // 成功构造
    explicit Result(T value) : value_(value), error_(Error::OK), has_value_(true) {}

    // 错误构造
    explicit Result(Error err) : value_{}, error_(err), has_value_(false) {}

    // 默认构造 = 错误
    Result() : value_{}, error_(Error::INVALID_STATE), has_value_(false) {}

    // 拷贝/移动
    Result(const Result&) = default;
    Result(Result&&) = default;
    Result& operator=(const Result&) = default;
    Result& operator=(Result&&) = default;

    // 查询接口
    [[nodiscard]] bool IsOk() const { return has_value_; }
    [[nodiscard]] bool IsErr() const { return !has_value_; }
    [[nodiscard]] explicit operator bool() const { return has_value_; }

    // 获取值 (错误时返回默认值)
    T ValueOr(T default_val) const {
        return has_value_ ? value_ : default_val;
    }

    // 获取值 (错误时断言失败)
    T& Value() {
        ASSERT(has_value_, "Result::Value() called on error");
        return value_;
    }

    const T& Value() const {
        ASSERT(has_value_, "Result::Value() called on error");
        return value_;
    }

    // 解引用操作符
    T* operator->() {
        ASSERT(has_value_, "Result::operator->() called on error");
        return &value_;
    }

    const T* operator->() const {
        ASSERT(has_value_, "Result::operator->() called on error");
        return &value_;
    }

    T& operator*() {
        ASSERT(has_value_, "Result::operator*() called on error");
        return value_;
    }

    const T& operator*() const {
        ASSERT(has_value_, "Result::operator*() called on error");
        return value_;
    }

    // 获取错误码
    [[nodiscard]] Error ErrorCode() const { return error_; }

    // 错误转换
    template<typename F>
    Result<T> MapError(F&& f) const {
        if (has_value_) return Result<T>(value_);
        return Result<T>(f(error_));
    }

    // 链式调用: 成功时执行下一个操作
    template<typename F>
    auto AndThen(F&& f) -> Result<std::invoke_result_t<F, T>> {
        if (!has_value_) return Result<std::invoke_result_t<F, T>>(error_);
        return f(value_);
    }

    // 错误处理: 错误时执行恢复操作
    template<typename F>
    Result<T> OrElse(F&& f) {
        if (has_value_) return *this;
        return f(error_);
    }

private:
    T value_;
    Error error_;
    bool has_value_;
};

// 特化 void 类型
template<>
class Result<void> {
public:
    Result() : error_(Error::OK), has_value_(true) {}
    explicit Result(Error err) : error_(err), has_value_(err == Error::OK) {}

    [[nodiscard]] bool IsOk() const { return has_value_; }
    [[nodiscard]] bool IsErr() const { return !has_value_; }
    [[nodiscard]] explicit operator bool() const { return has_value_; }
    [[nodiscard]] Error ErrorCode() const { return error_; }

private:
    Error error_;
    bool has_value_;
};

// ============================================================
// 便捷构造函数
// ============================================================

template<typename T>
[[nodiscard]] inline Result<T> Ok(T value) {
    return Result<T>(value);
}

[[nodiscard]] inline Result<void> Ok() {
    return Result<void>();
}

template<typename T = void>
[[nodiscard]] inline Result<T> Err(Error e) {
    return Result<T>(e);
}

// ============================================================
// 断言与调试
// ============================================================

#ifdef ENABLE_DEBUG_ASSERT
    #define ASSERT(cond, msg) \
        do { \
            if (!(cond)) { \
                error::ReportAssertionFailure(__FILE__, __LINE__, msg); \
                for (;;) {} \
            } \
        } while (0)
#else
    #define ASSERT(cond, msg) ((void)0)
#endif

// 检查返回值的宏
#define TRY(expr) \
    ({ \
        auto&& _res = (expr); \
        if (!_res.IsOk()) return _res; \
        std::move(_res).Value(); \
    })

// 带默认值的 TRY
#define TRY_OR(expr, default_val) \
    ({ \
        auto&& _res = (expr); \
        _res.IsOk() ? std::move(_res).Value() : (default_val); \
    })

// ============================================================
// 错误统计
// ============================================================

struct ErrorStats {
    uint32_t total_errors{0};
    uint32_t error_counts[16]{};
    Error last_error{Error::OK};
    uint32_t last_error_time{0};  // 由调用方填充时间戳
};

void RecordError(Error e);
const ErrorStats& GetErrorStats();
void ResetErrorStats();

// 报告断言失败 (内部使用)
void ReportAssertionFailure(const char* file, int line, const char* msg);

// ============================================================
// 错误处理器
// ============================================================

enum class RecoveryStrategy : uint8_t {
    IGNORE,         // 忽略错误，继续运行
    RETRY,          // 重试
    DEGRADE,        // 降级运行
    HALT,           // 停止当前模块
    REBOOT,         // 系统复位
};

using ErrorHook = void(*)(Error, const char* context);
using DelayHook = void(*)(uint32_t ms);  // 延迟回调，注入 FreeRTOS 或 HAL 延迟

struct ErrorHandlerConfig {
    RecoveryStrategy strategy{RecoveryStrategy::RETRY};
    uint8_t max_retries{3};
    uint32_t retry_delay_ms{10};
    ErrorHook on_error{nullptr};
    ErrorHook on_recovery{nullptr};
    DelayHook delay_hook{nullptr};  // 如果为 null，重试时不延迟
};

class ErrorHandler {
public:
    explicit ErrorHandler(const char* name, const ErrorHandlerConfig& cfg = {});

    // 处理错误，返回是否应该继续
    // 注意：如需重试延迟，确保 cfg.delay_hook 已设置（如 osDelay）
    bool Handle(Error e, const char* context = nullptr);

    // 重置状态
    void Reset();

    // 查询状态
    bool IsHealthy() const { return consecutive_errors_ == 0; }
    uint8_t GetConsecutiveErrors() const { return consecutive_errors_; }

private:
    const char* name_;
    ErrorHandlerConfig cfg_;
    uint8_t consecutive_errors_{0};
    uint32_t last_retry_delay_ms_{0};
};

// ============================================================
// C++17 兼容的构造守卫
// ============================================================

// 检测是否有 IsValid() 成员函数（C++17 兼容）
template<typename, typename = void>
struct has_is_valid : std::false_type {};

template<typename T>
struct has_is_valid<T, std::void_t<decltype(std::declval<T>().IsValid())>>
    : std::true_type {};

template<typename T, typename... Args>
Result<T*> ConstructGuarded(T* ptr, Args&&... args) {
    if (!ptr) return Err<T*>(Error::NO_MEMORY);

    // placement new 构造
    new (ptr) T(std::forward<Args>(args)...);

    // 检查构造后状态
    if constexpr (has_is_valid<T>::value) {
        if (!ptr->IsValid()) {
            ptr->~T();
            return Err<T*>(Error::INVALID_STATE);
        }
    }

    return Ok(ptr);
}

} // namespace error

// ============================================================
// 内存分配错误检查封装
// ============================================================

#define CHECK_NEW(ptr) \
    do { \
        if (!(ptr)) { \
            error::RecordError(error::Error::NO_MEMORY); \
            ASSERT(false, "Memory allocation failed: " #ptr); \
        } \
    } while (0)

#define SAFE_NEW(type, var, ...) \
    type* var = new (std::nothrow) type(__VA_ARGS__); \
    if (!(var)) return error::Err<type*>(error::Error::NO_MEMORY)

#define RETRY_NEW(type, var, max_retry, ...) \
    type* var = nullptr; \
    for (int _i = 0; _i < (max_retry); ++_i) { \
        var = new (std::nothrow) type(__VA_ARGS__); \
        if (var) break; \
    } \
    if (!(var)) return error::Err<type*>(error::Error::NO_MEMORY)
