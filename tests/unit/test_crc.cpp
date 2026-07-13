#include <gtest/gtest.h>

#include "referee_protocol.hpp"
#include "vision_protocol.hpp"

#include <cstdint>

namespace {

// ---- referee CRC8 (poly 0x31, init 0xFF) ----

TEST(RefereeCrc8, EmptyInputReturnsInit) {
    uint8_t dummy[1] = {0};
    EXPECT_EQ(referee::Crc8Calc(dummy, 0), 0xFF);
}

TEST(RefereeCrc8, HeaderRoundtrip) {
    // 构造 5B 帧头: SOF, data_length(2B), seq, CRC8
    uint8_t header[5] = {0xA5, 0x0B, 0x00, 0x01, 0x00};
    header[4] = referee::Crc8Calc(header, 4);
    EXPECT_TRUE(referee::Crc8Verify(header, 5));
}

TEST(RefereeCrc8, CorruptedHeaderFails) {
    uint8_t header[5] = {0xA5, 0x0B, 0x00, 0x01, 0x00};
    header[4] = referee::Crc8Calc(header, 4);
    header[1] ^= 0x01;  // 翻转 data_length 一位
    EXPECT_FALSE(referee::Crc8Verify(header, 5));
}

TEST(RefereeCrc8, ZeroLengthVerifyFails) {
    uint8_t dummy[1] = {0};
    EXPECT_FALSE(referee::Crc8Verify(dummy, 0));
}

// ---- referee CRC16 (poly 0x8005, init 0xFFFF) ----

TEST(RefereeCrc16, EmptyInputReturnsInit) {
    uint8_t dummy[1] = {0};
    EXPECT_EQ(referee::Crc16Calc(dummy, 0), 0xFFFF);
}

TEST(RefereeCrc16, AppendThenVerifyRoundtrip) {
    uint8_t frame[16] = {0xA5, 0x07, 0x00, 0x01, 0xDE,
                         0x01, 0x02, 0x11, 0x22, 0x33,
                         0x44, 0x55, 0x66, 0x77, 0x00, 0x00};
    referee::Crc16Append(frame, sizeof(frame));
    EXPECT_TRUE(referee::Crc16Verify(frame, sizeof(frame)));
}

TEST(RefereeCrc16, CorruptedPayloadFails) {
    uint8_t frame[16] = {};
    referee::Crc16Append(frame, sizeof(frame));
    frame[7] ^= 0x80;
    EXPECT_FALSE(referee::Crc16Verify(frame, sizeof(frame)));
}

TEST(RefereeCrc16, TooShortVerifyFails) {
    uint8_t two[2] = {0xAA, 0xBB};
    EXPECT_FALSE(referee::Crc16Verify(two, 2));
}

// ---- vision CRC16 (CRC-CCITT 反转多项式 0x8408) ----

TEST(VisionCrc16, AppendThenVerifyRoundtrip) {
    uint8_t frame[32] = {0xFF, 0x01, 0x02, 0x03};
    frame[31] = 0x0D;
    vision::Crc16Append(frame, 30);  // 视觉帧 CRC 位于 [28..29]
    EXPECT_TRUE(vision::Crc16Verify(frame, 30));
}

TEST(VisionCrc16, CorruptedPayloadFails) {
    uint8_t frame[30] = {0xFF, 0x01, 0x02, 0x03};
    vision::Crc16Append(frame, 30);
    frame[2] ^= 0x01;
    EXPECT_FALSE(vision::Crc16Verify(frame, 30));
}

// ---- 两套算法必须不同: 防止 include 串用 ----

TEST(CrcCrossCheck, RefereeAndVisionDiffer) {
    const uint8_t sample[8] = {0x11, 0x22, 0x33, 0x44,
                               0x55, 0x66, 0x77, 0x88};
    EXPECT_NE(referee::Crc16Calc(sample, 8), vision::Crc16Calc(sample, 8));
}

}  // namespace
