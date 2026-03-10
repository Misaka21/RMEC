#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>

// ---- 编译器屏障 ----
// Cortex-M4 单核无乱序执行, 仅需阻止 GCC 重排
#define TOPIC_COMPILER_BARRIER() __asm volatile("" ::: "memory")

// --------------------------------------------------------------------------
// TopicReader<T>  —— 每个订阅者持有一个实例, 独立追踪已读序号
// --------------------------------------------------------------------------
template <typename T>
class TopicReader {
public:
    // 有新数据且 SeqLock 一致则拷贝到 out, 返回 true
    bool Read(T& out) const {
        uint32_t s1 = *seq_ptr_;
        TOPIC_COMPILER_BARRIER();
        if (s1 & 1u) return false;       // 写入中
        if (s1 == last_seq_) return false; // 无新数据
        out = *data_ptr_;
        TOPIC_COMPILER_BARRIER();
        uint32_t s2 = *seq_ptr_;
        if (s1 != s2) return false;       // 被写者打断
        last_seq_ = s1;
        return true;
    }

    // 无条件返回最新值引用 (非关键场景, 可能读到半更新数据)
    const T& Latest() const { return *data_ptr_; }

    // 是否有未读数据
    bool HasNew() const { return *seq_ptr_ != last_seq_; }

private:
    template <typename, uint8_t>
    friend class Topic;

    const T*                  data_ptr_ = nullptr;
    const volatile uint32_t*  seq_ptr_  = nullptr;
    mutable uint32_t          last_seq_ = 0;
};

// --------------------------------------------------------------------------
// Topic<T, MaxSubs>  —— 单生产者, 多消费者, SeqLock 无锁发布订阅
// --------------------------------------------------------------------------
// 约束:
//   - T 必须 trivially copyable (保证赋值等价 memcpy)
//   - 单生产者: SeqLock 写端不可重入, 同一 Topic 仅一个任务/ISR 调用 Publish
//   - Subscribe() 仅在调度器启动前的初始化阶段调用
//   - Latest-value 语义: 无队列, 只保留最新值
// --------------------------------------------------------------------------
template <typename T, uint8_t MaxSubs = 8>
class Topic {
    static_assert(std::is_trivially_copyable_v<T>,
                  "Topic<T>: T must be trivially copyable");

public:
    // SeqLock 写入, 可在 ISR 或 Task 中调用 (单生产者)
    void Publish(const T& msg) {
        ++seq_;                     // → 奇数: 写入中
        TOPIC_COMPILER_BARRIER();
        data_ = msg;
        TOPIC_COMPILER_BARRIER();
        ++seq_;                     // → 偶数: 写入完成
    }

    // 分配一个 reader, 返回指针; 仅在初始化时调用
    TopicReader<T>* Subscribe() {
        if (count_ >= MaxSubs) return nullptr;
        auto& r     = readers_[count_++];
        r.data_ptr_ = &data_;
        r.seq_ptr_  = &seq_;
        r.last_seq_ = 0;
        return &r;
    }

    // 当前订阅者数
    uint8_t SubCount() const { return count_; }

private:
    T                  data_{};
    volatile uint32_t  seq_    = 0;
    TopicReader<T>     readers_[MaxSubs]{};
    uint8_t            count_  = 0;
};

#undef TOPIC_COMPILER_BARRIER
