# 测试基建 Phase A 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 `tests/` 宿主机测试工程与首批 L1 纯逻辑单测，并接入 GitHub Actions host-test 阻断式检查。

**Architecture:** `tests/` 是独立 CMake 工程（本机编译器），通过相对路径编译 `Core/` 中无 HAL 依赖的被测源文件，GoogleTest 经 FetchContent 引入。根构建与 `Core/` 目标代码零改动。

**Tech Stack:** CMake ≥3.16、GoogleTest v1.15.2（FetchContent 锁 tag）、GitHub Actions（ubuntu-latest）、C++17。

## Global Constraints

- 设计文档：`docs/internal/design/testing-infrastructure.md`（v1.0）
- 本计划全程不修改 `Core/` 与根 `CMakeLists.txt` 的任何文件
- 工作分支：`chore/test-infra-design`（延续 Draft PR #1），仓库根：`/home/wuji/work/sandbox/RMEC`
- **每条命令必须以 `cd /home/wuji/work/sandbox/RMEC &&` 开头**：本会话 shell cwd 会被重置到并行会话的 worktree，绝不能在裸 cwd 下执行 git/cmake
- 提交格式 gitmoji：`<emoji> <type>[scope]: <描述>`，不加 Co-Authored-By
- C++ 命名遵循项目 CLAUDE.md（测试函数名可用 GoogleTest 惯例）

## 偏离设计说明（评审时确认）

1. **Phase 边界比设计第 6.1 节保守**：`test_loop_queue`、`test_pid`、`test_float2str`、`test_dwt_math` 的断言目标是 P0 分支（`worktree-fix+audit-p0`）引入的修复行为，P0 未合入 main 前无稳定基线，全部移入 Phase B。本计划只含 P0 未触碰模块：CRC、DT7、mit_codec、power_limiter。
2. **CI 仅先启用 host-test job**：设计第 7 节的 `build-firmware` 依赖 P0 的 CMake 大小写修复（当前 main 在 Linux 上无法配置），`clang-tidy` 依赖交叉编译产出的 compile_commands.json，`size-check` 依赖 build-firmware 产物。三者与三板型矩阵、板型宏守卫一并放入 Phase B 计划。

---

### Task 1: tests/ 工程骨架与冒烟测试

**Files:**
- Create: `tests/CMakeLists.txt`
- Create: `tests/unit/test_sanity.cpp`

**Interfaces:**
- Produces: `rmec_add_test(name src...)` CMake 函数，后续 Task 全部用它注册测试；`RMEC_CORE` 变量指向 `Core/`

- [ ] **Step 1: 写冒烟测试**

`tests/unit/test_sanity.cpp`：

```cpp
#include <gtest/gtest.h>

TEST(Sanity, GoogleTestRuns) {
    EXPECT_EQ(1 + 1, 2);
}
```

- [ ] **Step 2: 写 tests/CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(rmec_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(RMEC_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../Core)

enable_testing()
include(GoogleTest)

# 注册一个测试目标: rmec_add_test(<name> <src>...)
# 被测 Core/ 源文件显式列出, 不 glob —— 哪些代码 host 可编译是显式契约
function(rmec_add_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE GTest::gtest_main)
    target_include_directories(${name} PRIVATE
        ${RMEC_CORE}/module
        ${RMEC_CORE}/module/referee
        ${RMEC_CORE}/module/vision
        ${RMEC_CORE}/module/remote/protocol
        ${RMEC_CORE}/module/motor
        ${RMEC_CORE}/module/motor/driver
        ${RMEC_CORE}/module/algorithm
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    gtest_discover_tests(${name})
endfunction()

rmec_add_test(test_sanity unit/test_sanity.cpp)
```

- [ ] **Step 3: 配置并构建，验证冒烟测试通过**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake -B build-tests -S tests && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure
```

预期输出结尾：`100% tests passed, 0 tests failed out of 1`

- [ ] **Step 4: 确认 build-tests 已被忽略**

```bash
cd /home/wuji/work/sandbox/RMEC && git status --short
```

预期：只出现 `tests/` 的新文件。若 `build-tests/` 出现在未跟踪列表，在 `.gitignore` 追加一行 `build-tests/`（这是对根目录唯一允许的改动，因为它不属于 `Core/` 与根构建）。

- [ ] **Step 5: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ .gitignore && git commit -m "✨ feat(tests): 搭建宿主机测试工程骨架"
```

---

### Task 2: CRC 单测（referee 与 vision 两套算法）

**Files:**
- Create: `tests/unit/test_crc.cpp`
- Modify: `tests/CMakeLists.txt`（追加一行注册）

**Interfaces:**
- Consumes: `referee::Crc8Calc/Crc8Verify/Crc16Calc/Crc16Verify/Crc16Append`（`referee_protocol.hpp`，header-only），`vision::Crc16Calc/Crc16Verify/Crc16Append`（`vision_protocol.hpp`，header-only）

- [ ] **Step 1: 写失败的测试**

`tests/unit/test_crc.cpp`：

```cpp
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
```

- [ ] **Step 2: 注册测试目标**

在 `tests/CMakeLists.txt` 末尾 `rmec_add_test(test_sanity ...)` 之后追加：

```cmake
rmec_add_test(test_crc unit/test_crc.cpp)
```

- [ ] **Step 3: 构建并运行，验证全部通过**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R Crc
```

预期：11 个用例全部 PASS。注意这批是"表征现有正确行为"的测试，写完即绿是预期结果，重点在防回归。若有 FAIL，先核对测试自身的字节构造再怀疑被测代码。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(referee,vision): 添加两套 CRC 校验单测"
```

---

### Task 3: DT7 协议解码单测

**Files:**
- Create: `tests/unit/test_dt7_protocol.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `remote::Dt7Protocol::Decode(const uint8_t*, Dt7Data&, const Dt7Data&)`、`remote::Dt7Protocol::Reset(Dt7Data&)`（编译 `Core/module/remote/protocol/dt7_protocol.cpp`），`remote::Dt7Data`、`remote::SwitchPos`、`remote::CH_OFFSET/CH_MAX`、`remote::Dt7Key/Dt7KeyMode`

- [ ] **Step 1: 写测试（含帧打包 helper，为 Decode 的精确逆运算）**

`tests/unit/test_dt7_protocol.cpp`：

```cpp
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
```

- [ ] **Step 2: 注册测试目标（编入被测 .cpp）**

`tests/CMakeLists.txt` 追加：

```cmake
rmec_add_test(test_dt7_protocol
    unit/test_dt7_protocol.cpp
    ${RMEC_CORE}/module/remote/protocol/dt7_protocol.cpp
)
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R Dt7
```

预期：9 个用例全部 PASS。若 `ChannelsDecodeExactly` 失败，用失败值反查 FrameBuilder 的位打包是否与 `dt7_protocol.cpp:8-12` 的解码公式互逆。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(remote): 添加 DT7 协议解码单测"
```

---

### Task 4: mit_codec 单测

**Files:**
- Create: `tests/unit/test_mit_codec.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `mit_codec::FloatToUint/UintToFloat/PackMitFrame/PackModeFrame`（`mit_codec.hpp`，header-only）

- [ ] **Step 1: 写测试**

`tests/unit/test_mit_codec.cpp`：

```cpp
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
```

注意：`FloatToUint` 的 `span == 0` 除零问题是审查报告已知缺陷，属 P1 修复范围，本任务不为当前 UB 行为写断言。

- [ ] **Step 2: 注册测试目标**

`tests/CMakeLists.txt` 追加：

```cmake
rmec_add_test(test_mit_codec unit/test_mit_codec.cpp)
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R Mit
```

预期：4 个用例全部 PASS。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(motor): 添加 MIT 编解码单测"
```

---

### Task 5: PowerLimiter 不变量单测

**Files:**
- Create: `tests/unit/test_power_limiter.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `PowerLimiter::Init(const PowerLimiterConfig&)`、`UpdateEnergyLoop(float, float, float, float)`、`Limit(PowerMotorState*, uint8_t)`、`GetMaxPower()`（`power_limiter.hpp/.cpp`，依赖 `rls2.hpp` 与 `math.hpp`，均无 HAL）

- [ ] **Step 1: 写测试（不变量风格：有限性与限幅，不断言内部算法数值）**

`tests/unit/test_power_limiter.cpp`：

```cpp
#include <gtest/gtest.h>

#include "power_limiter.hpp"

#include <cmath>
#include <cstdint>

namespace {

PowerLimiterConfig M3508Config() {
    PowerLimiterConfig cfg{};
    cfg.model.output_to_torque = (20.0f / 16384.0f) * 0.3f * (3591.0f / 187.0f);
    return cfg;
}

PowerMotorState MakeState(float out) {
    PowerMotorState s{};
    s.pid_output = out;
    s.speed_rad = 20.0f;
    s.set_speed_rad = 30.0f;
    s.max_output = 16384.0f;
    return s;
}

TEST(PowerLimiter, EnergyLoopProducesFiniteMaxPower) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.001f);
    EXPECT_TRUE(std::isfinite(pl.GetMaxPower()));
    EXPECT_GT(pl.GetMaxPower(), 0.0f);
}

TEST(PowerLimiter, ZeroDtDoesNotProduceNan) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.0f);
    EXPECT_TRUE(std::isfinite(pl.GetMaxPower()));
}

TEST(PowerLimiter, LimitKeepsOutputsFiniteAndClamped) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.001f);
    PowerMotorState states[4] = {
        MakeState(16000.0f), MakeState(-16000.0f),
        MakeState(8000.0f), MakeState(0.0f),
    };
    pl.Limit(states, 4);
    for (const auto& s : states) {
        EXPECT_TRUE(std::isfinite(s.pid_output));
        EXPECT_LE(std::fabs(s.pid_output), s.max_output);
    }
}

TEST(PowerLimiter, ZeroSpeedMotorsDoNotCrash) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 0.0f, 0.0f, 0.001f);  // 缓冲能量耗尽
    PowerMotorState states[2] = {MakeState(16000.0f), MakeState(16000.0f)};
    states[0].speed_rad = 0.0f;
    states[1].speed_rad = -0.001f;
    pl.Limit(states, 2);
    for (const auto& s : states) {
        EXPECT_TRUE(std::isfinite(s.pid_output));
    }
}

}  // namespace
```

- [ ] **Step 2: 注册测试目标（编入两个被测 .cpp）**

`tests/CMakeLists.txt` 追加：

```cmake
rmec_add_test(test_power_limiter
    unit/test_power_limiter.cpp
    ${RMEC_CORE}/module/motor/power_limiter.cpp
)
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R PowerLimiter
```

预期：4 个用例全部 PASS。若 `ZeroDtDoesNotProduceNan` 失败，说明 `power_limiter.cpp` 的 `inv_dt` 保护被移除过——这是真回归，报告而不是改测试。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(motor): 添加功率限制器不变量单测"
```

---

### Task 6: GitHub Actions host-test 工作流

**Files:**
- Create: `.github/workflows/ci.yml`

**Interfaces:**
- Produces: `host-test` job，Phase B 在同一文件追加 `build-firmware`/`clang-tidy`/`size-check` job

- [ ] **Step 1: 写工作流**

`.github/workflows/ci.yml`：

```yaml
name: CI

on:
  push:
    branches: ["**"]
  pull_request:

jobs:
  host-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Cache GoogleTest
        uses: actions/cache@v4
        with:
          path: build-tests/_deps
          key: gtest-${{ hashFiles('tests/CMakeLists.txt') }}

      - name: Configure
        run: cmake -B build-tests -S tests

      - name: Build
        run: cmake --build build-tests -j

      - name: Test
        run: ctest --test-dir build-tests --output-on-failure
```

- [ ] **Step 2: 本地最终全量验证**

```bash
cd /home/wuji/work/sandbox/RMEC && rm -rf build-tests && cmake -B build-tests -S tests && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure
```

预期输出结尾：`100% tests passed, 0 tests failed out of 29`（1 sanity + 11 crc + 9 dt7 + 4 mit + 4 power_limiter）

- [ ] **Step 3: 提交并推送**

```bash
cd /home/wuji/work/sandbox/RMEC && git add .github/ && git commit -m "👷 ci: 添加宿主机测试工作流" && git push
```

- [ ] **Step 4: 验证 CI 运行通过**

```bash
cd /home/wuji/work/sandbox/RMEC && gh run watch --exit-status $(gh run list --branch chore/test-infra-design --limit 1 --json databaseId --jq '.[0].databaseId')
```

预期：exit code 0，`host-test` job 绿。若失败，用 `gh run view --log-failed` 取日志定位。

- [ ] **Step 5: 在仓库设置将 host-test 设为 required check（用户操作）**

GitHub → Settings → Branches → main 保护规则 → Require status checks → 勾选 `host-test`。此步骤需要仓库管理员在网页完成，执行者在 PR 里留言提醒即可。

---

## 完成定义

- [ ] 6 个 Task 全部提交，`git log` 呈现 6 个独立提交
- [ ] `git diff main -- Core/ CMakeLists.txt` 为空（目标代码零改动）
- [ ] CI 在 PR #1 上显示 `host-test` 绿
- [ ] 设计文档验收标准 1 满足（干净环境一次通过）；标准 2/3/4 其余部分由 Phase B 计划覆盖
