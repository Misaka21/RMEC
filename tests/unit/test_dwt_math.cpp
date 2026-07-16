#include <gtest/gtest.h>

#include "main.h"
#include "sal_dwt.h"

#include <cstdint>

namespace {

// DwtInit 连调两次可将静态状态完全归零:
// 第一次把 CYCCNT 与 DwtCntUpdate 内部的 CYCCNT_LAST 都带回 0
// (可能产生一次假回绕计数), 第二次在全 0 基础上把 cyc_round_cnt_ 复位。
// 这让每个用例既能在独立进程 (ctest 逐用例过滤) 也能在同进程 (直接跑二进制)
// 下得到确定的初始状态。
void ResetDwt() {
    DwtInstance::DwtInit(168);
    DwtInstance::DwtInit(168);
}

// 断言口径: 复刻 DwtSysTimeUpdate 的整数分解 (对 168 MHz 逐级取整)
uint64_t ExpectedUs(uint64_t total_cycles) {
    constexpr uint64_t HZ = 168000000, HZ_MS = 168000, HZ_US = 168;
    uint64_t s   = total_cycles / HZ;
    uint64_t rem = total_cycles - s * HZ;
    uint64_t ms  = rem / HZ_MS;
    uint64_t us  = (rem - ms * HZ_MS) / HZ_US;
    return s * 1000000 + ms * 1000 + us;
}

TEST(DwtMath, TimelineBasic) {
    ResetDwt();
    fake_dwt.CYCCNT = 168000000u;  // 1 s
    EXPECT_EQ(DwtInstance::DwtGetTimeline_us(), 1000000u);
}

TEST(DwtMath, OverflowSpanIsTwoToThe32) {
    ResetDwt();
    fake_dwt.CYCCNT = 0x80000000u;
    DwtInstance::DwtGetTimeline_us();  // 推进 CYCCNT_LAST 到高位
    // 回绕到 80: 2^32 mod 168 == 88, 因此 (2^32 + 80) 恰好被 168 整除。
    // 正确实现: (2^32 + 80) / 168 = 25565282 us 整。
    // 若溢出跨度被改回 UINT32_MAX(2^32-1): (2^32 - 1 + 80) / 168 向下取整
    // = 25565281, 差恰好 1 us —— 以 1 cycle 精度区分两种实现。
    fake_dwt.CYCCNT = 80u;
    EXPECT_EQ(DwtInstance::DwtGetTimeline_us(), 25565282u);
}

TEST(DwtMath, TimelineUsSurvivesLongUptime) {
    // 回归目标: dwt_time_.s * 1000000 若按 uint32 计算, 连续运行约 71.6 分钟
    // (s >= 4295) 后返回值回绕。制造 200 次溢出 ≈ 5113 s, 越过回绕点。
    ResetDwt();
    const uint64_t wraps = 200;
    for (uint64_t i = 0; i < wraps; ++i) {
        fake_dwt.CYCCNT = 0x80000000u;
        DwtInstance::DwtGetTimeline_us();
        fake_dwt.CYCCNT = 0x00000000u;
        DwtInstance::DwtGetTimeline_us();
    }
    fake_dwt.CYCCNT = 80u;
    uint64_t total = (wraps << 32) + fake_dwt.CYCCNT;
    EXPECT_EQ(DwtInstance::DwtGetTimeline_us(), ExpectedUs(total));
}

TEST(DwtMath, DeltaTSurvivesWrap) {
    ResetDwt();
    // (uint32_t)(now - last) 的模运算在回绕时仍给出正确间隔
    DwtInstance dwt;
    fake_dwt.CYCCNT = 0xFFFFFF00u;
    dwt.DwtGetDeltaT();             // 设定基准
    fake_dwt.CYCCNT = 0x00000100u;  // 回绕后 +0x200 cycle
    float dt = dwt.DwtGetDeltaT();
    EXPECT_NEAR(dt, 512.0f / 168000000.0f, 1e-9f);
}

}  // namespace
