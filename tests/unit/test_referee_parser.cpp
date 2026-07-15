#include <gtest/gtest.h>

#include "referee.hpp"
#include "referee_protocol.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

namespace {

using referee::RefereeData;
using referee::RefereeParser;

std::vector<RefereeData>& Published() {
    static std::vector<RefereeData> v;
    return v;
}

void CapturePublish(const RefereeData& d) { Published().push_back(d); }

// 组一条完整合法帧: [SOF][len(2)][seq][CRC8][cmd(2)][payload][CRC16]
std::vector<uint8_t> MakeFrame(uint16_t cmd_id, const uint8_t* payload, uint16_t len) {
    std::vector<uint8_t> f(referee::HEADER_LEN + referee::CMD_ID_LEN + len +
                           referee::CRC16_LEN);
    f[0] = referee::SOF;
    f[1] = len & 0xFF;
    f[2] = (len >> 8) & 0xFF;
    f[3] = 0;
    f[4] = referee::Crc8Calc(f.data(), 4);
    f[5] = cmd_id & 0xFF;
    f[6] = (cmd_id >> 8) & 0xFF;
    if (len) std::memcpy(f.data() + 7, payload, len);
    referee::Crc16Append(f.data(), static_cast<uint16_t>(f.size()));
    return f;
}

std::vector<uint8_t> RobotStatusFrame(uint8_t robot_id, uint16_t heat_limit) {
    referee::WireRobotStatus w{};
    w.robot_id = robot_id;
    w.shooter_barrel_heat_limit = heat_limit;
    uint8_t payload[13]{};
    std::memcpy(payload, &w, sizeof(payload));
    return MakeFrame(referee::CMD_ROBOT_STATUS, payload, sizeof(payload));
}

class RefereeParserTest : public ::testing::Test {
protected:
    void SetUp() override { Published().clear(); }
    RefereeParser parser_{static_cast<RefereeParser::SendFunc>(nullptr),
                          CapturePublish};
};

TEST_F(RefereeParserTest, ValidFramePublishes) {
    auto f = RobotStatusFrame(7, 400);
    parser_.Parse(f.data(), static_cast<uint16_t>(f.size()));
    ASSERT_EQ(Published().size(), 1u);
    EXPECT_EQ(Published()[0].robot_status.robot_id, 7);
    EXPECT_EQ(Published()[0].robot_status.shooter_barrel_heat_limit, 400);
    EXPECT_EQ(parser_.RobotId(), 7);
}

TEST_F(RefereeParserTest, ByteByByteDeliveryStillParses) {
    auto f = RobotStatusFrame(3, 200);
    for (uint8_t b : f) parser_.Parse(&b, 1);
    EXPECT_EQ(Published().size(), 1u);
}

TEST_F(RefereeParserTest, TwoFramesInOneBuffer) {
    auto f1 = RobotStatusFrame(1, 100);
    auto f2 = RobotStatusFrame(2, 150);
    f1.insert(f1.end(), f2.begin(), f2.end());
    parser_.Parse(f1.data(), static_cast<uint16_t>(f1.size()));
    ASSERT_EQ(Published().size(), 2u);
    EXPECT_EQ(Published()[1].robot_status.robot_id, 2);
}

TEST_F(RefereeParserTest, BadHeaderCrc8DroppedThenResyncs) {
    auto bad = RobotStatusFrame(5, 300);
    bad[4] ^= 0xFF;  // 毁 CRC8
    parser_.Parse(bad.data(), static_cast<uint16_t>(bad.size()));
    EXPECT_TRUE(Published().empty());
    auto good = RobotStatusFrame(5, 300);
    parser_.Parse(good.data(), static_cast<uint16_t>(good.size()));
    EXPECT_EQ(Published().size(), 1u);  // 状态机已重同步
}

TEST_F(RefereeParserTest, BadFrameCrc16Dropped) {
    auto f = RobotStatusFrame(5, 300);
    f[8] ^= 0x01;  // 毁 payload → CRC16 失败
    parser_.Parse(f.data(), static_cast<uint16_t>(f.size()));
    EXPECT_TRUE(Published().empty());
}

TEST_F(RefereeParserTest, OversizeLengthRejectedThenResyncs) {
    // data_length 使 frame_len > 512: 帧头 CRC8 合法但长度越界
    uint8_t hdr[5] = {referee::SOF, 0xF8, 0x01, 0, 0};  // len=504 → frame=513
    hdr[4] = referee::Crc8Calc(hdr, 4);
    parser_.Parse(hdr, 5);
    EXPECT_TRUE(Published().empty());
    auto good = RobotStatusFrame(9, 250);
    parser_.Parse(good.data(), static_cast<uint16_t>(good.size()));
    EXPECT_EQ(Published().size(), 1u);
}

TEST_F(RefereeParserTest, WrongLengthForKnownCmdDropped) {
    // 0x0201 固定 13B, 伪造 12B 帧: CRC 全对但长度校验拒收
    uint8_t payload[12]{};
    auto f = MakeFrame(referee::CMD_ROBOT_STATUS, payload, sizeof(payload));
    parser_.Parse(f.data(), static_cast<uint16_t>(f.size()));
    EXPECT_TRUE(Published().empty());
}

TEST_F(RefereeParserTest, SendBuildsVerifiableFrame) {
    static std::vector<uint8_t> sent;
    sent.clear();
    RefereeParser tx_parser([](uint8_t* buf, uint16_t len) {
        sent.assign(buf, buf + len);
    });
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    tx_parser.Send(0x0301, payload, sizeof(payload));
    ASSERT_EQ(sent.size(), 7u + 4u + 2u);
    EXPECT_TRUE(referee::Crc8Verify(sent.data(), referee::HEADER_LEN));
    EXPECT_TRUE(referee::Crc16Verify(sent.data(), static_cast<uint16_t>(sent.size())));
}

}  // namespace
