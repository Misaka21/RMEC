#include <gtest/gtest.h>

#include "xStruct.hpp"

#include <cstdint>

namespace {

TEST(LoopQueue, FifoOrder) {
    LoopQueue<int> q(4);
    EXPECT_TRUE(q.push(1));
    EXPECT_TRUE(q.push(2));
    EXPECT_TRUE(q.push(3));
    int v = 0;
    EXPECT_TRUE(q.pop(v));
    EXPECT_EQ(v, 1);
    EXPECT_TRUE(q.pop(v));
    EXPECT_EQ(v, 2);
    EXPECT_EQ(q.size(), 1);
}

TEST(LoopQueue, PushOnFullQueueRejected) {
    LoopQueue<int> q(2);
    EXPECT_TRUE(q.push(1));
    EXPECT_TRUE(q.push(2));
    EXPECT_FALSE(q.push(3));
    EXPECT_EQ(q.size(), 2);
}

TEST(LoopQueue, PopOnEmptyReturnsFalse) {
    LoopQueue<int> q(2);
    int v = 42;
    EXPECT_FALSE(q.pop(v));
    EXPECT_EQ(v, 42);  // 输出参数不被改写
}

// 回归: 空队列 popout 曾使 size_ 下溢到 65535
TEST(LoopQueue, PopoutOnEmptyIsNoOp) {
    LoopQueue<int> q(2);
    q.popout();
    EXPECT_EQ(q.size(), 0);
    EXPECT_TRUE(q.empty());
    // 下溢会让后续 push 全部失败, 验证队列仍可用
    EXPECT_TRUE(q.push(7));
    int v = 0;
    EXPECT_TRUE(q.pop(v));
    EXPECT_EQ(v, 7);
}

TEST(LoopQueue, ValuePopOnEmptyReturnsDefault) {
    LoopQueue<int> q(2);
    EXPECT_EQ(q.pop(), 0);
    EXPECT_EQ(q.size(), 0);
}

TEST(LoopQueue, WrapAround) {
    LoopQueue<int> q(3);
    int v = 0;
    for (int i = 0; i < 10; ++i) {
        EXPECT_TRUE(q.push(i));
        EXPECT_TRUE(q.pop(v));
        EXPECT_EQ(v, i);
    }
    EXPECT_TRUE(q.empty());
}

TEST(LoopQueue, ClearResets) {
    LoopQueue<int> q(3);
    q.push(1);
    q.push(2);
    q.clear();
    EXPECT_TRUE(q.empty());
    EXPECT_TRUE(q.push(9));
    EXPECT_EQ(q.front(), 9);
}

}  // namespace
