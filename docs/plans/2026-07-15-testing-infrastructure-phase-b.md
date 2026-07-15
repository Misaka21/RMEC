# 测试基建 Phase B 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成设计文档 v1.0 的剩余范围：fake_hal/fake_sal 支撑层、6.1 剩余 L1 用例（float2str、dwt_math、referee_parser）、6.2 全部 L3 数据流用例、板型宏守卫与 CI 扩展（三板型矩阵 / size-check / clang-tidy）。

**Architecture:** 延续 Phase A 的 `tests/` 独立 CMake 工程。新增 `tests/support/fake_hal/`（同名假 CubeMX/HAL 头，include 优先级覆盖真头）与 `tests/support/fake_sal/`（链接期替换 `sal_*.cc` 真实现 + 假 TaskManager 步进框架）。L3 测试编译真实 module/app 源文件，通过 fake CAN/UART 注入与断言。

**Tech Stack:** CMake ≥3.16、GoogleTest v1.15.2（已有）、GitHub Actions（ubuntu-latest, apt gcc-arm-none-eabi, clang-tidy）、C++17。

## Global Constraints

- 设计文档：`docs/internal/design/testing-infrastructure.md`（v1.0）
- 对 `Core/` 的唯一允许改动：`robot_def.hpp` 板型宏 `#ifndef` 守卫（设计第 7 节声明，约 3 行）
- 工作分支：`chore/test-infra-phase-b`，仓库根：`/home/wuji/work/sandbox/RMEC`
- 每条命令以 `cd /home/wuji/work/sandbox/RMEC &&` 开头
- 提交格式 gitmoji：`<emoji> <type>[scope]: <描述>`，不加 Co-Authored-By
- 测试目标全部经 `rmec_add_test` / 新增 `rmec_add_fake_test` 注册，被测源显式列出
- 本机无 arm-none-eabi-gcc 与 clang-tidy：Task 8/9 的交叉编译与 tidy 只能靠 CI 验证，推送后用 `gh run watch` 闭环

## 偏离设计说明（评审时确认）

1. **build-firmware 工具链沿用 apt 安装**（设计 7 节建议 arm 官方 pinned release）：Phase A/P0 合入的 apt 方案已在 main 上跑绿，Phase B 只加矩阵不换工具链，pinned 迁移另行处理。
2. **size-check 数据源用链接期 `--print-memory-usage` 输出**（根 CMakeLists 已启用该链接选项）而非解析 `.map`：同样能取到三个 region 的用量与百分比，且零新增工具。
3. **fake TaskManager 用同名头覆盖**（`tests/support/fake_hal/TaskManager.hpp`）：真 TaskManager 的 `osThreadCreate`/`vTaskDelayUntil` 无法在 host 运行，设计 4 节"假 TaskManager 只记录 init/task"即此实现。
4. **L3 flow 测试静态状态跨用例共享**：设计 6.2 约束"每个任务簇一个测试二进制"，用例按声明顺序执行、靠发布新数据驱动迁移，不做用例间隔离。

---

### Task 1: fake 层地基 + test_float2str

**Files:**
- Create: `tests/support/fake_hal/usart.h`
- Create: `tests/support/fake_hal/main.h`
- Create: `tests/support/fake_hal/cmsis_os.h`
- Create: `tests/support/fake_sal/fake_sal_test_api.hpp`
- Create: `tests/support/fake_sal/fake_registers.cc`
- Create: `tests/support/fake_sal/fake_sal_usart.cc`
- Create: `tests/unit/test_float2str.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `rmec_add_fake_test(name src...)` CMake 函数（fake_hal include 优先 + 自动链入 fake_sal 支撑源）；`test::SentUartPackets()/InjectUartRx()/ResetFakeRecords()`；假寄存器 `fake_dwt/fake_core_debug` 与假句柄 `huart1/3/6`
- Consumes: `Float2Str(char*, float)`（`Core/sal/log/log.cc`，编译整个 TU，链接依赖 fake_sal_usart 提供的 `sal::UartInstance` 符号）

- [ ] **Step 1: 写 fake_hal 三个最小假头**

`tests/support/fake_hal/usart.h`：

```c
#pragma once
// 假 CubeMX usart.h: 仅提供 sal_usart.h / log.h 在宿主机编译所需的最小类型
#include <stdint.h>

typedef struct UART_HandleTypeDef {
    int id;  // 测试用标识, 区分不同串口
} UART_HandleTypeDef;

#define HAL_MAX_DELAY 0xFFFFFFFFU

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
```

`tests/support/fake_hal/main.h`：

```c
#pragma once
// 假 CubeMX main.h: DWT/CoreDebug 寄存器打桩, 供 sal_dwt 在宿主机编译与测试注入
#include <stdint.h>

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} FakeDwtType;

typedef struct {
    volatile uint32_t DEMCR;
} FakeCoreDebugType;

extern FakeDwtType       fake_dwt;
extern FakeCoreDebugType fake_core_debug;

#define DWT (&fake_dwt)
#define CoreDebug (&fake_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0)
```

`tests/support/fake_hal/cmsis_os.h`：

```c
#pragma once
// 假 cmsis_os: 仅提供类型与常量, 线程创建被假 TaskManager 接管, 不会真正调用
typedef enum {
    osPriorityIdle        = -3,
    osPriorityLow         = -2,
    osPriorityBelowNormal = -1,
    osPriorityNormal      = 0,
    osPriorityAboveNormal = 1,
    osPriorityHigh        = 2,
    osPriorityRealtime    = 3,
} osPriority;

typedef void* osThreadId;
```

- [ ] **Step 2: 写 fake_sal 测试 API 头与实现**

`tests/support/fake_sal/fake_sal_test_api.hpp`：

```cpp
#pragma once
// fake_sal 测试侧 API: 断言发送、注入接收、控制时间 (设计文档 5.2 节)
#include <cstdint>
#include <vector>

struct UART_HandleTypeDef;
struct CAN_HandleTypeDef;

namespace test {

struct UartPacket {
    UART_HandleTypeDef* handle;
    std::vector<uint8_t> data;
};
std::vector<UartPacket>& SentUartPackets();
void InjectUartRx(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len);

struct CanFrame {
    CAN_HandleTypeDef* handle;
    uint32_t std_id;
    uint8_t data[8];
};
std::vector<CanFrame>& SentCanFrames();
void InjectCanRx(CAN_HandleTypeDef* hcan, uint32_t std_id, const uint8_t (&data)[8]);

// 推进假 DWT 时钟 (按 DwtInit(168) 的 168 MHz 折算)
void AdvanceTimeMs(uint32_t ms);

// 清空收发记录 (不清实例注册表; SAL 实例生命周期为全局)
void ResetFakeRecords();

}  // namespace test
```

`tests/support/fake_sal/fake_registers.cc`：

```cpp
// 假外设句柄与假寄存器的唯一定义
#include "main.h"
#include "usart.h"

FakeDwtType       fake_dwt{};
FakeCoreDebugType fake_core_debug{};

UART_HandleTypeDef huart1{1};
UART_HandleTypeDef huart3{3};
UART_HandleTypeDef huart6{6};
```

（`hcan1/hcan2` 在 Task 4 的 can.h 落地时再补，避免本 Task 引入未使用符号。）

`tests/support/fake_sal/fake_sal_usart.cc`（编译真 `sal_usart.h`，链接期替换真实现；成员函数在此定义因而拥有私有成员访问权，借此把注入通道登记到测试注册表）：

```cpp
#include "sal_usart.h"
#include "fake_sal_test_api.hpp"

#include <cstring>
#include <functional>
#include <vector>

namespace {

struct FakeUartEntry {
    UART_HandleTypeDef* handle;
    std::function<void(const uint8_t*, uint16_t)> inject;
};

std::vector<FakeUartEntry>& Registry() {
    static std::vector<FakeUartEntry> r;
    return r;
}

}  // namespace

namespace test {

std::vector<UartPacket>& SentUartPackets() {
    static std::vector<UartPacket> v;
    return v;
}

void InjectUartRx(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len) {
    for (auto& e : Registry())
        if (e.handle == huart) e.inject(data, len);
}

}  // namespace test

namespace sal {

std::vector<UartInstance::UartPtr> UartInstance::instance_list_{};

UartInstance::UartInstance(const UartConfig& config)
    : handle_(config.handle),
      rx_size_(config.rx_size),
      rx_buff_{},
      rx_type_(config.rx_type),
      rx_cbk_(config.rx_cbk),
      tx_use_fifo_(config.use_fifo),
      tx_queue_mx_size_(config.queue_mx_size),
      tx_type_(config.tx_type),
      tx_cbk_(config.tx_cbk) {
    instance_list_.push_back(this);
    Registry().push_back({handle_, [this](const uint8_t* data, uint16_t len) {
        uint16_t n = len < UART_MX_RX_BUFFER_SIZE ? len : UART_MX_RX_BUFFER_SIZE;
        std::memcpy(rx_buff_, data, n);
        if (rx_cbk_) rx_cbk_(rx_buff_, n);
    }});
}

UartTxState UartInstance::UartSend(uint8_t* data, uint16_t size, uint32_t) {
    test::SentUartPackets().push_back({handle_, {data, data + size}});
    if (tx_cbk_) tx_cbk_();
    return UartTxState::BLOCK_FINISH;
}

void UartInstance::UartSetSendType(UartTxType type, bool use_fifo, uint8_t queue_mx_size) {
    tx_type_ = type;
    tx_use_fifo_ = use_fifo;
    tx_queue_mx_size_ = queue_mx_size;
}

uint16_t UartInstance::UartRecv(uint8_t*, uint16_t, uint32_t) { return 0; }
void UartInstance::UartSetRecvType(UartRxType type, uint16_t size) {
    rx_type_ = type;
    rx_size_ = size;
}
void UartInstance::UartRestartRecv() {}
void UartInstance::UartStopRecv() {}
void UartInstance::PopSend() {}

}  // namespace sal
```

- [ ] **Step 3: 在 tests/CMakeLists.txt 增加 rmec_add_fake_test**

在 `rmec_add_test` 定义之后追加：

```cmake
# fake 支撑源: 假寄存器/句柄定义 + 链接期替换的假 sal 实现
set(RMEC_FAKE_SUPPORT
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_registers.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_sal_usart.cc
)

# 注册一个使用 fake_hal/fake_sal 的测试目标:
# fake_hal 目录以 BEFORE 优先, 让 Core/ 源里的 "usart.h"/"main.h"/"cmsis_os.h"/
# "TaskManager.hpp" 解析到假头; sal_*.h 用真头, 实现链接 fake_sal
function(rmec_add_fake_test name)
    rmec_add_test(${name} ${ARGN} ${RMEC_FAKE_SUPPORT})
    target_include_directories(${name} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_hal
        ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal
    )
    target_include_directories(${name} PRIVATE
        ${RMEC_CORE}/sal/usart
        ${RMEC_CORE}/sal/can
        ${RMEC_CORE}/sal/dwt
        ${RMEC_CORE}/sal/log
        ${RMEC_CORE}/module/daemon
        ${RMEC_CORE}/module/imu
        ${RMEC_CORE}/app
        ${RMEC_CORE}/app/robot_task
        ${RMEC_CORE}/app/motor_task
        ${RMEC_CORE}/app/remote_task
        ${RMEC_CORE}/app/vision_task
        ${RMEC_CORE}/app/referee_task
        ${RMEC_CORE}/app/ins_task
    )
endfunction()
```

末尾注册：

```cmake
rmec_add_fake_test(test_float2str
    unit/test_float2str.cpp
    ${RMEC_CORE}/sal/log/log.cc
)
```

- [ ] **Step 4: 写 test_float2str**

`tests/unit/test_float2str.cpp`：

```cpp
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
```

- [ ] **Step 5: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake -B build-tests -S tests && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R Float2Str
```

预期：5 个用例 PASS，且既有 40 个用例不受影响（`ctest` 全量仍 45 全绿）。

- [ ] **Step 6: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(sal): 添加 Float2Str 单测与 fake_hal/fake_sal 地基"
```

---

### Task 2: test_dwt_math（DWT 溢出跨度）

**Files:**
- Create: `tests/unit/test_dwt_math.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `DwtInstance::DwtInit/DwtGetTimeline_us/DwtGetDeltaT`（编译 `Core/sal/dwt/sal_dwt.cc`，寄存器经 fake main.h 注入）；`fake_dwt.CYCCNT` 可直接写

- [ ] **Step 1: 写测试**

`tests/unit/test_dwt_math.cpp`：

```cpp
#include <gtest/gtest.h>

#include "main.h"
#include "sal_dwt.h"

#include <cstdint>

namespace {

// 断言口径: 复刻 DwtSysTimeUpdate 的整数分解 (对 168 MHz 逐级取整)
uint64_t ExpectedUs(uint64_t total_cycles) {
    constexpr uint64_t HZ = 168000000, HZ_MS = 168000, HZ_US = 168;
    uint64_t s   = total_cycles / HZ;
    uint64_t rem = total_cycles - s * HZ;
    uint64_t ms  = rem / HZ_MS;
    uint64_t us  = (rem - ms * HZ_MS) / HZ_US;
    return s * 1000000 + ms * 1000 + us;
}

// 注意: DwtInstance 静态状态 (cyc_round_cnt_ 等) 与 DwtCntUpdate 内部的
// CYCCNT_LAST 跨用例共享, 本文件用例按声明顺序构成一条时间线, 不可乱序。

TEST(DwtMath, TimelineBasic) {
    DwtInstance::DwtInit(168);          // 复位 CYCCNT=0, round=0
    fake_dwt.CYCCNT = 168000000u;       // 1 s
    EXPECT_EQ(DwtInstance::DwtGetTimeline_us(), 1000000u);
}

TEST(DwtMath, OverflowSpanIsTwoToThe32) {
    // 制造 336 次溢出: 若溢出跨度按 UINT32_MAX(2^32-1) 计, 会累计少 336 cycle = 2 us
    const uint64_t wraps = 336;
    for (uint64_t i = 0; i < wraps; ++i) {
        fake_dwt.CYCCNT = 0x80000000u;
        DwtInstance::DwtGetTimeline_us();
        fake_dwt.CYCCNT = 0x00000000u;  // 回绕, cyc_round_cnt_++
        DwtInstance::DwtGetTimeline_us();
    }
    fake_dwt.CYCCNT = 42u * 168u;       // 42 us
    uint64_t total = (wraps << 32) + fake_dwt.CYCCNT;
    EXPECT_EQ(DwtInstance::DwtGetTimeline_us(), ExpectedUs(total));
}

TEST(DwtMath, DeltaTSurvivesWrap) {
    // (uint32_t)(now - last) 的模运算在回绕时仍给出正确间隔
    DwtInstance dwt;
    fake_dwt.CYCCNT = 0xFFFFFF00u;
    dwt.DwtGetDeltaT();                 // 设定基准
    fake_dwt.CYCCNT = 0x00000100u;      // 回绕后 +0x200 cycle
    float dt = dwt.DwtGetDeltaT();
    EXPECT_NEAR(dt, 512.0f / 168000000.0f, 1e-9f);
}

}  // namespace
```

- [ ] **Step 2: 注册**

```cmake
rmec_add_fake_test(test_dwt_math
    unit/test_dwt_math.cpp
    ${RMEC_CORE}/sal/dwt/sal_dwt.cc
)
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R DwtMath
```

预期：3 用例 PASS。若 `OverflowSpanIsTwoToThe32` 红，先核对 `sal_dwt.cc:40` 的 `<< 32` 是否被改回乘法。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(sal): 添加 DWT 溢出跨度与回绕单测"
```

---

### Task 3: test_referee_parser

**Files:**
- Create: `tests/unit/test_referee_parser.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `referee::RefereeParser::Parse/Send`（编译 `Core/module/referee/referee.cpp`；TU 内 `Referee` 包装类链接 fake_sal_usart）；`referee::Crc8Calc/Crc16Append`（header-only, 造帧用）；`CMD_ROBOT_STATUS=0x0201 (13B)`、`CMD_POWER_HEAT=0x0202 (14B)`

- [ ] **Step 1: 写测试（帧构造 helper + 6.1 规定的四类场景）**

`tests/unit/test_referee_parser.cpp`：

```cpp
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
```

- [ ] **Step 2: 注册**

```cmake
rmec_add_fake_test(test_referee_parser
    unit/test_referee_parser.cpp
    ${RMEC_CORE}/module/referee/referee.cpp
)
```

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R RefereeParser
```

预期：8 用例 PASS。若 `HEADER_LEN/SOF` 等常量名对不上，以 `referee_protocol.hpp` 实际定义为准修 include/命名，不改被测代码。

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(referee): 添加裁判系统协议解析器单测"
```

---

### Task 4: fake CAN + 假 TaskManager 步进框架

**Files:**
- Create: `tests/support/fake_hal/can.h`
- Create: `tests/support/fake_hal/TaskManager.hpp`
- Create: `tests/support/fake_sal/fake_sal_can.cc`
- Create: `tests/support/fake_sal/fake_task_manager.cc`
- Modify: `tests/support/fake_sal/fake_registers.cc`（补 hcan1/hcan2 定义）
- Modify: `tests/CMakeLists.txt`（`RMEC_FAKE_SUPPORT` 追加两个 .cc）

**Interfaces:**
- Produces: `test::SentCanFrames()/InjectCanRx()`（fake_sal_test_api.hpp 已声明）；`test::TaskRegistry()/RunTaskInit(name)/StepTask(name, n)`；`hcan1/hcan2` 假句柄
- Consumes: 真 `sal_can.h` 的 `CanInstance` 声明（私有成员在 fake 实现内可访问）

- [ ] **Step 1: 写 can.h 假头 + 句柄定义**

`tests/support/fake_hal/can.h`：

```c
#pragma once
// 假 CubeMX can.h: 仅提供 sal_can.h 在宿主机编译所需的最小类型
#include <stdint.h>

typedef struct CAN_HandleTypeDef {
    int id;  // 测试用标识, 区分总线
} CAN_HandleTypeDef;

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_TxHeaderTypeDef;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
```

`fake_registers.cc` 追加：

```cpp
#include "can.h"

CAN_HandleTypeDef hcan1{1};
CAN_HandleTypeDef hcan2{2};
```

- [ ] **Step 2: 写 fake_sal_can.cc**

```cpp
#include "sal_can.h"
#include "fake_sal_test_api.hpp"

#include <cstring>
#include <functional>
#include <vector>

namespace {

struct FakeCanEntry {
    CAN_HandleTypeDef* handle;
    uint32_t rx_id;
    std::function<void(const uint8_t*, uint8_t)> inject;
};

std::vector<FakeCanEntry>& Registry() {
    static std::vector<FakeCanEntry> r;
    return r;
}

}  // namespace

namespace test {

std::vector<CanFrame>& SentCanFrames() {
    static std::vector<CanFrame> v;
    return v;
}

void InjectCanRx(CAN_HandleTypeDef* hcan, uint32_t std_id, const uint8_t (&data)[8]) {
    for (auto& e : Registry())
        if (e.handle == hcan && e.rx_id == std_id) e.inject(data, 8);
}

}  // namespace test

namespace sal {

std::vector<CanInstance::CanPtr> CanInstance::instance_[CAN_DEV_NUM]{};
CanInstance::FifoPtr CanInstance::fifo_ptr_[CAN_DEV_NUM]{};
uint8_t CanInstance::can1_filter_idx_ = 0;
uint8_t CanInstance::can2_filter_idx_ = 0;

CanInstance::CanInstance(const CanConfig& config)
    : handle_(config.handle),
      tx_header_{},
      tx_mailbox_(0),
      tx_id_(config.tx_id),
      tx_len_(8),
      tx_cbk_(config.tx_cbk),
      fifo_timeout_us_(config.fifo_timeout_us),
      rx_id_(config.rx_id),
      rx_data_{},
      rx_cbk_(config.rx_cbk),
      busy_cnt_(0),
      wait_time_(0) {
    Registry().push_back({handle_, rx_id_, [this](const uint8_t* data, uint8_t len) {
        std::memcpy(rx_data_, data, len);
        if (rx_cbk_) rx_cbk_(len);
    }});
}

void CanInstance::CanServiceInit() {}
void CanInstance::CanAddFilter() {}

bool CanInstance::CanTransmit(const CanMsg& msg, uint16_t) {
    test::CanFrame f{handle_, tx_id_, {}};
    std::memcpy(f.data, msg.data, 8);
    test::SentCanFrames().push_back(f);
    if (tx_cbk_) tx_cbk_();
    return true;
}

void CanInstance::CanFifoxCallback(CAN_HandleTypeDef*, uint32_t) {}
void CanInstance::CanTxFinishCallback(CAN_HandleTypeDef*) {}
void CanInstance::TxQueueFront(FifoPtr&) {}
void CanInstance::PopQueueCallback(FifoPtr&) {}

}  // namespace sal
```

- [ ] **Step 3: 写假 TaskManager（同名头覆盖）**

`tests/support/fake_hal/TaskManager.hpp`：

```cpp
#pragma once
// 假 TaskManager: 不创建 FreeRTOS 线程, 只登记 init/task 到 test::TaskRegistry,
// 测试代码手动 RunTaskInit + StepTask 步进 (设计文档第 4 节, L3 不启动 FreeRTOS)
#include "cmsis_os.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace test {

struct TaskEntry {
    std::string name;
    uint32_t period_ms = 1;
    std::function<void()> init_func;
    std::function<void()> task_func;
};

std::vector<TaskEntry>& TaskRegistry();
void RunTaskInit(const char* name);
void StepTask(const char* name, uint32_t times = 1);

}  // namespace test

class TaskManager {
public:
    // 字段名与声明顺序必须与真 TaskManager::TaskConfig 一致 (app 层用指定初始化器)
    struct TaskConfig {
        const char*           name       = "default";
        uint32_t              stack_size = 128;
        osPriority            priority   = osPriorityNormal;
        uint32_t              period_ms  = 1;
        std::function<void()> init_func  = nullptr;
        std::function<void()> task_func  = nullptr;
    };

    TaskManager() = default;
    TaskManager(const TaskConfig& config);

    osThreadId task_handle = nullptr;
};
```

`tests/support/fake_sal/fake_task_manager.cc`：

```cpp
#include "TaskManager.hpp"

#include <cstring>

namespace test {

std::vector<TaskEntry>& TaskRegistry() {
    static std::vector<TaskEntry> r;
    return r;
}

void RunTaskInit(const char* name) {
    for (auto& t : TaskRegistry())
        if (t.name == name && t.init_func) t.init_func();
}

void StepTask(const char* name, uint32_t times) {
    for (auto& t : TaskRegistry())
        if (t.name == name && t.task_func)
            for (uint32_t i = 0; i < times; ++i) t.task_func();
}

}  // namespace test

TaskManager::TaskManager(const TaskConfig& config) {
    test::TaskRegistry().push_back(
        {config.name, config.period_ms, config.init_func, config.task_func});
    task_handle = this;
}
```

- [ ] **Step 4: AdvanceTimeMs 落位**

`fake_registers.cc` 追加（假 DWT 时钟推进；168 与 `DwtInit(168)` 约定一致）：

```cpp
namespace test {
void AdvanceTimeMs(uint32_t ms) { fake_dwt.CYCCNT += ms * 168000u; }
void ResetFakeRecords();  // 定义见下
}
```

`ResetFakeRecords`（清空 uart/can 发送记录）放在 `fake_registers.cc`，实现为调用两个 `Sent*()` 的 `clear()`。

- [ ] **Step 5: CMake 更新并全量构建**

`RMEC_FAKE_SUPPORT` 追加 `fake_sal_can.cc`、`fake_task_manager.cc`。

```bash
cd /home/wuji/work/sandbox/RMEC && cmake -B build-tests -S tests && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure
```

预期：既有用例全绿（本 Task 无新用例，是 L3 的地基）。

- [ ] **Step 6: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✨ feat(tests): fake CAN 桩与任务步进框架"
```

---

### Task 5: L3 test_shoot_loader

**Files:**
- Create: `tests/flow/test_shoot_loader.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `ShootMotors::Init/Tick`（编译 `shoot_motors.cpp` + `dji_driver.cpp` + `cascade_pid.cpp` + `pid_controller.cpp`）；`shoot_cmd_topic`；`test::SentCanFrames/InjectCanRx`；DJI 反馈 `rx_id = 0x200 + motor_id`（M3508/M2006），发送组 CAN2 0x1FF（摩擦轮 id5/6 → 帧内偏移 0/2，拨弹 id7 → 偏移 4）

**场景（设计 6.2）：** SINGLE/TRIPLE 边沿只走一发、REVERSE、热量保护停转。

- [ ] **Step 1: 写测试**

`tests/flow/test_shoot_loader.cpp` 关键结构（完整代码执行时按编译结果微调字节偏移）：

```cpp
#include <gtest/gtest.h>

#include "shoot_motors.hpp"
#include "robot_def.hpp"
#include "dji_driver.hpp"
#include "fake_sal_test_api.hpp"

#include <cstdint>

namespace {

ShootMotors& Shoot() {
    static ShootMotors s;
    return s;
}

// 读取最近一次 FlushAll 中 CAN2 0x1FF 帧里指定电机的 int16 输出
bool LastShootGroupOutput(uint8_t byte_offset, int16_t* out) {
    for (auto it = test::SentCanFrames().rbegin();
         it != test::SentCanFrames().rend(); ++it) {
        if (it->handle == &hcan2 && it->std_id == 0x1FF) {
            *out = static_cast<int16_t>((it->data[byte_offset] << 8) |
                                        it->data[byte_offset + 1]);
            return true;
        }
    }
    return false;
}

void PublishAndTick(ShootCmdData cmd, int ticks = 1) {
    for (int i = 0; i < ticks; ++i) {
        shoot_cmd_topic.Publish(cmd);
        Shoot().Tick(0.001f);
        DjiDriver::FlushAll();
    }
}

constexpr uint8_t FRICTION_L_OFF = 0;  // id5 → (5-5)*2
constexpr uint8_t FRICTION_R_OFF = 2;  // id6
constexpr uint8_t LOADER_OFF     = 4;  // id7

TEST(ShootLoader, InitOnce) { Shoot().Init(); SUCCEED(); }

TEST(ShootLoader, FrictionOffAllZero) { ... }          // OFF → 三个偏移全 0
TEST(ShootLoader, FrictionOnOppositeSpin) { ... }      // ON → L>0, R<0
TEST(ShootLoader, ReverseSpinsNegative) { ... }        // REVERSE → loader < 0
TEST(ShootLoader, SingleShotArmsOnceOnEdge) { ... }    // STOP→SINGLE 边沿输出>0;
                                                       // 注入反馈转到位后输出归零;
                                                       // 继续 SINGLE 不再重新武装
TEST(ShootLoader, HeatZeroStopsLoaderAndDropsTarget) { ... }
                                                       // rest_heat=0 → loader 输出 0;
                                                       // 热量恢复后不补射
}  // namespace
```

断言写法约定：每个用例开头 `test::ResetFakeRecords()`；模式序列依赖 `last_load_mode_` 静态状态，用例声明顺序即时间线。反馈注入用 `test::InjectCanRx(&hcan2, 0x200 + LOADER_MOTOR_ID, frame)`，帧格式按 `DjiDriver::DecodeFeedback` 实际布局（编码 ecd/rpm，执行时从 `dji_driver.cpp` 读出精确字节序）。

- [ ] **Step 2: 注册**

```cmake
rmec_add_fake_test(test_shoot_loader
    flow/test_shoot_loader.cpp
    ${RMEC_CORE}/app/motor_task/shoot_motors.cpp
    ${RMEC_CORE}/module/motor/driver/dji_driver.cpp
    ${RMEC_CORE}/module/motor/cascade_pid.cpp
    ${RMEC_CORE}/module/algorithm/pid_controller.cpp
    ${RMEC_CORE}/sal/log/log.cc
)
```

（`log.cc` 因 `dji_driver.cpp` 的 `DEBUG_DEADLOCK→PrintLog` 引用而链入。）

- [ ] **Step 3: 构建运行**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R ShootLoader
```

- [ ] **Step 4: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add tests/ && git commit -m "✅ test(app): 发射拨弹状态机数据流测试"
```

---

### Task 6: L3 test_robot_task_modes

**Files:**
- Create: `tests/flow/test_robot_task_modes.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `RobotTaskStart()`（编译 `robot_task.cpp`，任务经假 TaskManager 登记为 `"robot"`）；`remote_topic/vision_topic/gimbal_feed_topic/referee_topic` 发布；`gimbal_cmd_topic/chassis_cmd_topic/shoot_cmd_topic` 订阅断言
- Produces: 测试内 fake 联机开关 `g_remote_online/g_ins_ready/g_vision_online/g_referee_online`——`RemoteIsOnline()/InsIsReady()/VisionIsOnline()/RefereeIsOnline()/VisionSetMode()` 的桩定义（链接期替换各 task TU 的实现）

**场景（设计 6.2）：** 拨杆三位 × 模式迁移全路径、70 Hz 发布 + 200 Hz 消费不抖动、遥控离线 ZERO_FORCE、IMU 未就绪门控、视觉接管、拨轮→装填模式、热量锁存。

- [ ] **Step 1: 写测试（fake 桩 + 场景用例）**

测试文件顶部定义桩：

```cpp
#include "remote_task.hpp"
#include "vision_task.hpp"
#include "ins_task.hpp"
#include "referee_task.hpp"

static bool g_remote_online  = false;
static bool g_ins_ready      = true;
static bool g_vision_online  = false;
static bool g_referee_online = false;
static vision::AimMode g_last_aim = vision::AimMode::OFF;

bool RemoteIsOnline() { return g_remote_online; }
bool InsIsReady() { return g_ins_ready; }
bool VisionIsOnline() { return g_vision_online; }
bool RefereeIsOnline() { return g_referee_online; }
void VisionSetMode(vision::AimMode m, vision::EnemyColor, float) { g_last_aim = m; }
Dt7Remote* GetRemote() { return nullptr; }
referee::RefereeParser* GetRefereeParser() { return nullptr; }
```

用例序列（声明顺序执行）：

1. `StartRegistersTask`：`RobotTaskStart(); test::RunTaskInit("robot")` → TaskRegistry 出现 `"robot"`
2. `OfflineForcesZeroForce`：offline + sw_r=MID → gimbal/chassis 均 ZERO_FORCE
3. `SwitchDownIsZeroForce`：online + DOWN → ZERO_FORCE
4. `SwitchMidIsGyroNoFollow`：online + MID + ins ready → GYRO_MODE / NO_FOLLOW
5. `InsNotReadyBlocksGyro`：MID + !ins_ready → gimbal ZERO_FORCE、chassis 仍 NO_FOLLOW
6. `SlowRemoteDoesNotFlap`：发布一帧 MID 后连续 StepTask 4 次 → 每次都 GYRO_MODE（70/200Hz 保持帧回归）
7. `InvalidSwitchZeroIsZeroForce`：sw_r 原始值 0 → ZERO_FORCE（非法拨杆消费侧防御）
8. `VisionTakeoverSetsAbsoluteTargets`：UP + vision online + control=1 → yaw=vision.yaw×RAD_2_DEGREE；aim mode = AUTO_AIM
9. `DialControlsLoader`：sw_l=UP, dial<-330 → BURST；dial>330 → REVERSE；|dial|≤330 → STOP
10. `HeatLatch`：referee online（limit=100, heat=30）→ rest_heat=70；referee 掉线 → 仍 70；（首用例即断言从未上线时=255，放在序列早期，编号 2.5 处）

- [ ] **Step 2: 注册**

```cmake
rmec_add_fake_test(test_robot_task_modes
    flow/test_robot_task_modes.cpp
    ${RMEC_CORE}/app/robot_task/robot_task.cpp
)
```

- [ ] **Step 3: 构建运行 + 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R RobotTask && git add tests/ && git commit -m "✅ test(app): robot_task 模式状态机数据流测试"
```

---

### Task 7: L3 test_topic_wiring

**Files:**
- Create: `tests/flow/test_topic_wiring.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MotorTaskStart()`（ONE_BOARD：chassis+gimbal+shoot 全编）；步进 `"motor"` 任务；`gimbal_cmd/chassis_cmd/shoot_cmd/ins_topic` 发布；`test::SentCanFrames` 断言；`DwtInstance::DwtInit(168)` + `test::AdvanceTimeMs`
- 编译源：`motor_task.cpp, chassis_motors.cpp, gimbal_motors.cpp, shoot_motors.cpp, dji_driver.cpp, dm_driver.cpp, ht_driver.cpp, lk_driver.cpp, cascade_pid.cpp, pid_controller.cpp, power_limiter.cpp, sal_dwt.cc, log.cc`

**场景（设计 6.2）：** 每个 cmd topic 发布后步进消费任务，断言 fake CAN 收到非零输出（拦截"断头链路"）。

- [ ] **Step 1: 写测试**

结构：

```cpp
TEST(TopicWiring, MotorTaskRegisters) {
    DwtInstance::DwtInit(168);
    MotorTaskStart();                  // 创建全部电机 (ONE_BOARD)
    test::RunTaskInit("motor");
}

TEST(TopicWiring, GimbalCmdReachesCan1) {
    // GYRO_MODE + 非零目标 → yaw GM6020(id1) 电压帧 CAN1 0x1FF 偏移 0 非零
}

TEST(TopicWiring, ChassisCmdReachesCan2) {
    // NO_FOLLOW + vx≠0 → CAN2 0x200 至少一个轮子输出非零
}

TEST(TopicWiring, ShootCmdReachesCan2) {
    // friction ON → CAN2 0x1FF 摩擦轮偏移非零
}
```

每个用例：`test::ResetFakeRecords()` → 发布 cmd → `test::AdvanceTimeMs(1); test::StepTask("motor")` → 检索 `SentCanFrames`。若某链路依赖电机在线/反馈（执行时按 `chassis_motors.cpp`/`gimbal_motors.cpp` 逻辑确认），用 `InjectCanRx` 喂一帧反馈再步进。

- [ ] **Step 2: 注册（源列表见 Interfaces）+ 构建运行 + 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure -R TopicWiring && git add tests/ && git commit -m "✅ test(app): Topic 链路连通性数据流测试"
```

---

### Task 8: 板型宏守卫 + CI 三板型矩阵与 size-check

**Files:**
- Modify: `Core/app/robot_def.hpp:3-7`（唯一 Core 改动）
- Modify: `.github/workflows/ci.yml`

- [ ] **Step 1: 板型宏守卫**

`robot_def.hpp` 第 3-7 行改为：

```cpp
// ======================== 板型 (三选一) ========================
// 默认 ONE_BOARD; 构建系统可用 -DCHASSIS_BOARD / -DGIMBAL_BOARD 注入覆盖 (CI 矩阵用)

#if !defined(ONE_BOARD) && !defined(CHASSIS_BOARD) && !defined(GIMBAL_BOARD)
#define ONE_BOARD
#endif
```

（下方"恰好一个"的 `#error` 冲突守卫保留。）

- [ ] **Step 2: ci.yml 改造 build-firmware 为矩阵 + 上传产物 + size-check job**

```yaml
  build-firmware:
    runs-on: ubuntu-latest
    strategy:
      fail-fast: false
      matrix:
        board: [ONE_BOARD, CHASSIS_BOARD, GIMBAL_BOARD]
    steps:
      - uses: actions/checkout@v4
      - name: Install ARM Toolchain
        run: sudo apt-get update && sudo apt-get install -y gcc-arm-none-eabi ninja-build
      - name: Configure
        run: >
          cmake -B build -G Ninja
          -DCMAKE_C_FLAGS=-D${{ matrix.board }}
          -DCMAKE_CXX_FLAGS=-D${{ matrix.board }}
      - name: Build
        run: set -o pipefail && cmake --build build 2>&1 | tee build.log
      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        with:
          name: firmware-${{ matrix.board }}
          path: |
            build/pwf_fw.elf
            build/pwf_fw.hex
            build/pwf_fw.bin
            build/pwf_fw.map
            build.log

  size-check:
    runs-on: ubuntu-latest
    needs: build-firmware
    steps:
      - uses: actions/download-artifact@v4
        with:
          pattern: firmware-*
      - name: Check memory usage (阈值 90%)
        run: |
          python3 - <<'EOF'
          import glob, re, sys
          THRESHOLD = 90.0
          rows, failed = [], False
          for log in sorted(glob.glob('firmware-*/build.log')):
              board = log.split('/')[0].removeprefix('firmware-')
              for m in re.finditer(r'^\s*(FLASH|RAM|CCMRAM):\s+(\S+ \S+)\s+(\S+ \S+)\s+([\d.]+)%',
                                   open(log).read(), re.M):
                  region, used, total, pct = m.group(1), m.group(2), m.group(3), float(m.group(4))
                  mark = '❌' if pct > THRESHOLD else '✅'
                  failed |= pct > THRESHOLD
                  rows.append(f'| {board} | {region} | {used} | {total} | {pct:.2f}% {mark} |')
          with open(os.environ... ) # 写 $GITHUB_STEP_SUMMARY 表格
          sys.exit(1 if failed else 0)
          EOF
```

（执行时补全 summary 写入；正则以 `--print-memory-usage` 的真实输出格式为准。）

- [ ] **Step 3: 提交（两个提交）**

```bash
cd /home/wuji/work/sandbox/RMEC && git add Core/app/robot_def.hpp && git commit -m "🔧 chore(app): 板型宏改为可注入的 ifndef 守卫"
cd /home/wuji/work/sandbox/RMEC && git add .github/ && git commit -m "👷 ci: build-firmware 三板型矩阵与 size-check"
```

---

### Task 9: CI clang-tidy 增量检查

**Files:**
- Modify: `.github/workflows/ci.yml`

- [ ] **Step 1: 增加 clang-tidy job**

```yaml
  clang-tidy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - name: Install toolchain
        run: sudo apt-get update && sudo apt-get install -y gcc-arm-none-eabi ninja-build clang-tidy
      - name: Configure (生成 compile_commands.json)
        run: cmake -B build -G Ninja
      - name: Collect changed Core files
        id: changed
        run: |
          if [ "${{ github.event_name }}" = "pull_request" ]; then
            BASE="origin/${{ github.base_ref }}"
          else
            BASE="origin/main"
          fi
          git fetch origin main --quiet || true
          git diff --name-only --diff-filter=ACMR "$BASE"...HEAD -- 'Core/**/*.cpp' 'Core/**/*.cc' > changed.txt || true
          echo "count=$(wc -l < changed.txt)" >> "$GITHUB_OUTPUT"
          cat changed.txt
      - name: Run clang-tidy (bugprone/concurrency/performance)
        if: steps.changed.outputs.count != '0'
        run: |
          # clang 不认识部分 gcc 专属交叉编译旗标, 从编译数据库剔除
          sed -i 's/-mthumb-interwork //g; s/-fmessage-length=0 //g' build/compile_commands.json
          # 把 arm-none-eabi-g++ 的系统头路径喂给 clang
          EXTRA_ARGS="--extra-arg=--target=arm-none-eabi --extra-arg=-Wno-unknown-warning-option"
          for d in $(arm-none-eabi-g++ -xc++ /dev/null -E -Wp,-v 2>&1 | sed -n 's/^ //p'); do
            EXTRA_ARGS="$EXTRA_ARGS --extra-arg=-isystem --extra-arg=$d"
          done
          xargs -a changed.txt clang-tidy -p build \
            --checks='-*,bugprone-*,concurrency-*,performance-*' \
            --warnings-as-errors='*' $EXTRA_ARGS
```

- [ ] **Step 2: 提交**

```bash
cd /home/wuji/work/sandbox/RMEC && git add .github/ && git commit -m "👷 ci: 增量 clang-tidy 检查 (bugprone/concurrency/performance)"
```

---

### Task 10: 全量验证、推送、PR 与 CI 闭环

- [ ] **Step 1: 本地全量验证**

```bash
cd /home/wuji/work/sandbox/RMEC && rm -rf build-tests && cmake -B build-tests -S tests && cmake --build build-tests -j && ctest --test-dir build-tests --output-on-failure
```

预期：Phase A 40 例 + 本计划新增全部通过，0 failed。

- [ ] **Step 2: Core 零改动核查**

```bash
cd /home/wuji/work/sandbox/RMEC && git diff main -- Core/ CMakeLists.txt
```

预期：仅 `Core/app/robot_def.hpp` 的板型宏守卫（约 3 行语义改动）。

- [ ] **Step 3: 推送并建 PR**

```bash
cd /home/wuji/work/sandbox/RMEC && git push -u origin chore/test-infra-phase-b && gh pr create --title "✅ test: 测试基建 Phase B——fake 层、L3 数据流测试与 CI 矩阵" --body "<按 commit 生成 + 自测结果 + 提醒管理员把 host-test/build-firmware×3/size-check/clang-tidy 设为 required checks>"
```

- [ ] **Step 4: 盯 CI 至全绿（失败则取日志修复、重推，循环）**

```bash
cd /home/wuji/work/sandbox/RMEC && gh run watch --exit-status $(gh run list --branch chore/test-infra-phase-b --limit 1 --json databaseId --jq '.[0].databaseId')
```

---

## 完成定义

- [ ] 设计 6.1 表中 9 个 L1 用例全部存在且通过（Phase A 6 个 + 本计划 3 个）
- [ ] 设计 6.2 表中 3 个 flow 二进制存在且通过
- [ ] `git diff main -- Core/` 仅含板型宏守卫
- [ ] CI 六个 job（host-test、build-firmware×3、size-check、clang-tidy）在 PR 上全绿
- [ ] PR 描述含自测结果与 required-check 设置提醒
