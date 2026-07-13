#include <gtest/gtest.h>

#include "mit_codec.hpp"

#include <cstdint>

namespace {

TEST(MitCodec, FloatToUintBoundaries) {
    // 只用 12-bit 边界: (25 * 4095) 在 float 24-bit 尾数内精确表示,
    // 16-bit 情况乘积超出精确范围, 截断结果依赖舍入方向, 不适合做精确断言
    EXPECT_EQ(mit_codec::FloatToUint(-12.5f, -12.5f, 12.5f, 12), 0);
    EXPECT_EQ(mit_codec::FloatToUint(12.5f, -12.5f, 12.5f, 12), 4095);
}

TEST(MitCodec, RoundtripWithinQuantizationError) {
    const float x_min = -12.5f, x_max = 12.5f;
    const int bits = 12;
    const float lsb = (x_max - x_min) / 4095.0f;  // ≈ 0.0061
    for (float x : {-12.5f, -7.3f, 0.0f, 1.0f, 12.49f}) {
        uint16_t u = mit_codec::FloatToUint(x, x_min, x_max, bits);
        float back = mit_codec::UintToFloat(u, x_min, x_max, bits);
        EXPECT_NEAR(back, x, lsb) << "x=" << x;
    }
}

TEST(MitCodec, PackMitFrameBitLayout) {
    // 位布局: pos[16] | vel[12] | kp[12] | kd[12] | torque[12]
    uint8_t buf[8] = {};
    mit_codec::PackMitFrame(buf, 0x1234, 0xABC, 0x123, 0xDEF, 0x456);
    EXPECT_EQ(buf[0], 0x12);
    EXPECT_EQ(buf[1], 0x34);
    EXPECT_EQ(buf[2], 0xAB);
    EXPECT_EQ(buf[3], 0xC1);
    EXPECT_EQ(buf[4], 0x23);
    EXPECT_EQ(buf[5], 0xDE);
    EXPECT_EQ(buf[6], 0xF4);
    EXPECT_EQ(buf[7], 0x56);
}

TEST(MitCodec, PackModeFrame) {
    uint8_t buf[8] = {};
    mit_codec::PackModeFrame(buf, 0xFC);
    for (int i = 0; i < 7; ++i) {
        EXPECT_EQ(buf[i], 0xFF) << "i=" << i;
    }
    EXPECT_EQ(buf[7], 0xFC);
}

}  // namespace
