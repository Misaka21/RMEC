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

/* 自旋位锁: 原子test-and-set实现, 任务与中断上下文均可安全调用try_lock
 * 注意: 抢不到锁不会阻塞等待, 调用方需自行处理失败分支 */
class BitLocker
{
    friend class LockGuard;

private:
    volatile uint8_t bit = 0;

public:
    BitLocker() = default;
    ~BitLocker() = default;

    // 原子交换: 旧值为0表示成功取锁。裸的"先读后写"在抢占/中断下会双方同时持锁
    bool try_lock()
    {
        return __atomic_exchange_n(&bit, 1, __ATOMIC_ACQUIRE) == 0;
    }

    void unlock()
    {
        __atomic_store_n(&bit, 0, __ATOMIC_RELEASE);
    }
};

/* 释放守卫: 仅负责析构时释放锁, 不负责取锁!
 * 必须在 try_lock() 成功后才能构造, 否则会误释放他人持有的锁 */
class LockGuard
{
private:
    BitLocker *locker_ = nullptr;

public:
    LockGuard(BitLocker &locker) : locker_(&locker){};
    ~LockGuard() { locker_->unlock(); };
    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;
};
