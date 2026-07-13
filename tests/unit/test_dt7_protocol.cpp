#include <gtest/gtest.h>

#include "dt7_protocol.hpp"

#include <cstdint>
#include <cstring>

namespace {

using remote::Dt7Data;
using remote::Dt7Protocol;
using remote::SwitchPos;

// Decode 位布局的精确逆运算: 把期望值打包成 18B DBUS 帧
struct FrameBuilder {
    int16_t ch_r_x = 0, ch_r_y = 0, ch_l_x = 0, ch_l_y = 0, dial = 0;
    uint8_t sw_r = 3, sw_l = 3;  // 默认 MID
    uint16_t keys = 0;
    int16_t mouse_x = 0, mouse_y = 0;
    uint8_t mouse_l = 0, mouse_r = 0;

    void Build(uint8_t (&buf)[18]) const {
        std::memset(buf, 0, sizeof(buf));
        const uint16_t c0 = static_cast<uint16_t>(ch_r_x + remote::CH_OFFSET);
        const uint16_t c1 = static_cast<uint16_t>(ch_r_y + remote::CH_OFFSET);
        const uint16_t c2 = static_cast<uint16_t>(ch_l_x + remote::CH_OFFSET);
        const uint16_t c3 = static_cast<uint16_t>(ch_l_y + remote::CH_OFFSET);
        const uint16_t c4 = static_cast<uint16_t>(dial + remote::CH_OFFSET);
        buf[0]  = static_cast<uint8_t>(c0 & 0xFF);
        buf[1]  = static_cast<uint8_t>(((c0 >> 8) & 0x07) | ((c1 & 0x1F) << 3));
        buf[2]  = static_cast<uint8_t>(((c1 >> 5) & 0x3F) | ((c2 & 0x03) << 6));
        buf[3]  = static_cast<uint8_t>((c2 >> 2) & 0xFF);
        buf[4]  = static_cast<uint8_t>(((c2 >> 10) & 0x01) | ((c3 & 0x7F) << 1));
        buf[5]  = static_cast<uint8_t>(((c3 >> 7) & 0x0F) |
                                       ((sw_r & 0x03) << 4) | ((sw_l & 0x03) << 6));
        buf[6]  = static_cast<uint8_t>(mouse_x & 0xFF);
        buf[7]  = static_cast<uint8_t>((mouse_x >> 8) & 0xFF);
        buf[8]  = static_cast<uint8_t>(mouse_y & 0xFF);
        buf[9]  = static_cast<uint8_t>((mouse_y >> 8) & 0xFF);
        buf[12] = mouse_l;
        buf[13] = mouse_r;
        buf[14] = static_cast<uint8_t>(keys & 0xFF);
        buf[15] = static_cast<uint8_t>((keys >> 8) & 0xFF);
        buf[16] = static_cast<uint8_t>(c4 & 0xFF);
        buf[17] = static_cast<uint8_t>((c4 >> 8) & 0x07);
    }

    Dt7Data Decode(const Dt7Data& prev = Dt7Data{}) const {
        uint8_t buf[18];
        Build(buf);
        Dt7Data out{};
        Dt7Protocol::Decode(buf, out, prev);
        return out;
    }
};

TEST(Dt7Decode, CenterFrameDecodesToZero) {
    FrameBuilder fb;
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.ch_r_x, 0);
    EXPECT_EQ(d.ch_r_y, 0);
    EXPECT_EQ(d.ch_l_x, 0);
    EXPECT_EQ(d.ch_l_y, 0);
    EXPECT_EQ(d.dial, 0);
    EXPECT_EQ(d.sw_r, SwitchPos::MID);
    EXPECT_EQ(d.sw_l, SwitchPos::MID);
}

TEST(Dt7Decode, ChannelsDecodeExactly) {
    FrameBuilder fb;
    fb.ch_r_x = 300;
    fb.ch_r_y = -450;
    fb.ch_l_x = 660;
    fb.ch_l_y = -660;
    fb.dial = 123;
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.ch_r_x, 300);
    EXPECT_EQ(d.ch_r_y, -450);
    EXPECT_EQ(d.ch_l_x, 660);
    EXPECT_EQ(d.ch_l_y, -660);
    EXPECT_EQ(d.dial, 123);
}

TEST(Dt7Decode, OutOfRangeChannelRectifiedToZero) {
    FrameBuilder fb;
    fb.ch_r_x = 700;   // 超出 ±660, 11-bit 编码仍合法
    fb.ch_l_y = -700;
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.ch_r_x, 0);
    EXPECT_EQ(d.ch_l_y, 0);
}

TEST(Dt7Decode, SwitchPositions) {
    FrameBuilder fb;
    fb.sw_r = static_cast<uint8_t>(SwitchPos::UP);
    fb.sw_l = static_cast<uint8_t>(SwitchPos::DOWN);
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.sw_r, SwitchPos::UP);
    EXPECT_EQ(d.sw_l, SwitchPos::DOWN);
}

// 表征测试: 拨杆原始值 0 不属于任何 SwitchPos 枚举, Decode 原样透传。
// 消费端必须自行防御 (设计文档 6.2 节, robot_task 离线用例覆盖消费侧)。
// P1 若在协议层增加校验, 此用例需同步更新。
TEST(Dt7Decode, InvalidSwitchValueZeroPassesThrough) {
    FrameBuilder fb;
    fb.sw_r = 0;
    Dt7Data d = fb.Decode();
    EXPECT_EQ(static_cast<uint8_t>(d.sw_r), 0);
}

TEST(Dt7Decode, MouseAndButtons) {
    FrameBuilder fb;
    fb.mouse_x = -32700;
    fb.mouse_y = 12345;
    fb.mouse_l = 1;
    fb.mouse_r = 1;
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.mouse_x, -32700);
    EXPECT_EQ(d.mouse_y, 12345);
    EXPECT_EQ(d.mouse_l, 1);
    EXPECT_EQ(d.mouse_r, 1);
}

TEST(Dt7Decode, KeyRisingEdgeCountsOnce) {
    FrameBuilder fb;
    fb.keys = 1u << remote::KEY_W;
    Dt7Data prev{};                    // 上一帧无按键
    Dt7Data d1 = fb.Decode(prev);      // W 上升沿
    EXPECT_EQ(d1.key_count[remote::KEY_PRESS][remote::KEY_W], 1);
    Dt7Data d2 = fb.Decode(d1);        // W 持续按住, 不再计数
    EXPECT_EQ(d2.key_count[remote::KEY_PRESS][remote::KEY_W], 1);
}

TEST(Dt7Decode, CtrlComboCountsSeparately) {
    FrameBuilder fb;
    fb.keys = (1u << remote::KEY_Q) | (1u << remote::KEY_CTRL);
    Dt7Data d = fb.Decode();
    EXPECT_EQ(d.key_count[remote::KEY_PRESS_WITH_CTRL][remote::KEY_Q], 1);
    EXPECT_EQ(d.key_count[remote::KEY_PRESS][remote::KEY_Q], 0);
}

TEST(Dt7Reset, ZeroesEverything) {
    FrameBuilder fb;
    fb.ch_r_x = 100;
    fb.keys = 0xFFFF;
    Dt7Data d = fb.Decode();
    Dt7Protocol::Reset(d);
    EXPECT_EQ(d.ch_r_x, 0);
    EXPECT_EQ(d.keys, 0);
    EXPECT_EQ(static_cast<uint8_t>(d.sw_r), 0);
}

}  // namespace
