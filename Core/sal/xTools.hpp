#pragma once
// sal
#include "log.h"

#include <cstdint>
#include <memory>

#define DEBUG_DEADLOCK(message) \
    do                          \
    {                           \
        while (1)               \
            LOGERROR(message);  \
    } while (0)

/* 互斥锁,仅单核处理器可用 */
class BitLocker
{
    friend class LockGuard;

private:
    volatile uint8_t bit = 0;

public:
    BitLocker() = default;
    ~BitLocker() = default;

    bool try_lock()
    {
        if (!bit)
            return (bit = 1);
        else
            return false;
    }
};

/* 配合互斥锁,完成锁资源操作后自动析构释放锁 */
class LockGuard
{
private:
    volatile uint8_t *bit_ptr = nullptr;

public:
    LockGuard(BitLocker &locker) : bit_ptr(&locker.bit){};
    ~LockGuard() { *bit_ptr = 0; };
};