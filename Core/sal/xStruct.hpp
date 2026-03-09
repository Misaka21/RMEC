#pragma once

#include <stdint.h>
#include <vector>
#include <memory>

/**
 * @brief  开辟一块固定的内存大小,用头索引和尾索引实现循环队列
 *         相比queue更高效,但需要预先分配内存
 *
 * @attention 当队列已满,不允许新的push,会返回false
 *
 * @tparam T
 */
template <typename T>
class loop_queue
{
private:
    uint16_t max_size_ = 0;
    uint16_t size_ = 0;
    uint16_t front_ = 0;
    uint16_t rear_ = 0;
    std::unique_ptr<T[]> data_;

public:
    loop_queue(){};
    loop_queue(uint16_t size) : max_size_(size), data_(std::make_unique<T[]>(size)) {}
    ~loop_queue() { data_.reset(); } // 需要手动添加析构函数

    void resize(uint16_t size)
    {
        max_size_ = size;
        data_.reset();
        data_ = std::make_unique<T[]>(size);
        clear();
    }

    bool push(const T &data)
    {
        if (size_ >= max_size_)
            return false;
        data_[rear_] = data;
        rear_ = (rear_ + 1) % max_size_;
        ++size_;
        return true;
    }

    bool pop(T &data)
    {
        if (size_ == 0)
            return false;
        data = data_[front_];
        front_ = (front_ + 1) % max_size_;
        --size_;
        return true;
    }

    T pop()
    {
        front_ = (front_ + 1) % max_size_;
        --size_;
        return data_[front_ == 0 ? max_size_ - 1 : front_ - 1]; // 避免保存临时变量
    }

    T &front()
    {
        return data_[front_];
    }

    void front(T &data)
    {
        data = data_[front_];
    }

    T &back()
    {
        return data_[rear_];
    }

    void back(T &data)
    {
        data = data_[rear_];
    }

    void popout()
    {
        front_ = (front_ + 1) % max_size_;
        --size_;
    }

    void clear()
    {
        size_ = 0;
        front_ = 0;
        rear_ = 0;
    }

    bool empty() const
    {
        return size_ == 0;
    }

    bool size() const
    {
        return size_;
    }

    uint16_t max_size() const
    {
        return max_size_;
    }
};
