#include <gtest/gtest.h>

#include "log.h"

namespace {

// 用二进制精确可表示的值断言, 规避 float→int 截断的表示误差

TEST(Float2Str, PadsFractionToThreeDigits) {
    char buf[32];
    // 1 + 1/512 = 1.001953125 → 小数部分 1.953125 → 截断 1 → "001"
    // 回归目标: 若丢失零填充会输出 "1.1"
    Float2Str(buf, 1.001953125f);
    EXPECT_STREQ(buf, "1.001");
}

TEST(Float2Str, ExactHalf) {
    char buf[32];
    Float2Str(buf, 1.5f);
    EXPECT_STREQ(buf, "1.500");
}

TEST(Float2Str, Zero) {
    char buf[32];
    Float2Str(buf, 0.0f);
    EXPECT_STREQ(buf, "0.000");
}

TEST(Float2Str, NegativeWithIntegerPart) {
    char buf[32];
    Float2Str(buf, -2.75f);
    EXPECT_STREQ(buf, "-2.750");
}

TEST(Float2Str, NegativeFractionOnly) {
    // 整数部分为 0 时符号来自 flag 分支, 回归目标: "-0.250" 而非 "0.250"
    char buf[32];
    Float2Str(buf, -0.25f);
    EXPECT_STREQ(buf, "-0.250");
}

}  // namespace
