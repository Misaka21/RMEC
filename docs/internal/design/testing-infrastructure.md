# RMEC 测试基建设计

| 版本 | 日期 | 作者 | 变更说明 |
|------|------|------|---------|
| v1.0 | 2026-07-13 | Misaka21 | 首版：L0 CI、L1 宿主机单测、L3 数据流测试 |

关联工作项：无（经确认本次不关联飞书单号，后续补录）

## 1. 背景

全仓库代码审查（2026-07-13）发现约 60 项缺陷，其中 9 项为上车即暴露的功能性 bug。复盘结论：其中 7 项可被宿主机测试拦截，其余 2 项可被多配置编译或假 HAL 测试拦截。当前仓库没有任何测试和 CI，`-Wall -Werror` 与 clang-tidy 配置均依赖开发者手动执行，验证成本全部压在上车联调。

P0 缺陷修复正在 `worktree-fix+audit-p0` 分支并行进行（构建、SAL、module 层已完成，app 层进行中）。本设计基于该分支合入 main 后的基线。

## 2. 范围与非目标

**范围：**

- L0：GitHub Actions CI（交叉编译、clang-tidy、固件资源检查）
- L1：宿主机 GoogleTest 单测工程，为已修复 bug 补回归用例
- L3：假 sal 桩 + 手动步进任务，覆盖模式状态机与 Topic 数据流

**非目标（另行设计）：**

- P1 重构（SeqLock 收敛、原子锁原语、SPSC 队列、InstanceRegistry、SAL 错误码统一、IWDG）单独出一份设计，待本基建落地后编写，届时重构受回归测试保护
- L2 模块级假 HAL 深度测试（BMI088 寄存器表仿真等）按需追加，不在首批
- 仿真/HIL 不在本设计范围

## 3. 已确认决策

| 决策点 | 结论 | 理由 |
|--------|------|------|
| 范围切分 | 测试基建与 P1 重构拆两份设计 | 评审粒度小，重构落地时有回归保护 |
| SAL/HAL 替换策略 | 链接期替换 | 目标代码零改动，无运行时开销，与 P0 分支零冲突 |
| 工程组织 | `tests/` 独立 CMake 工程 | 根构建（CubeMX 生成）完全不动，增量交付 |
| GoogleTest 引入 | FetchContent 锁定 tag（v1.15.x） | CI 有网络，避免 13 MB 级源码入库 |
| CI 门槛 | 全部阻断式 required check | clang-tidy 规则从小集合起步，避免存量告警海 |

## 4. 总体架构

```
RMEC/
├── .github/workflows/ci.yml        # L0：单一工作流，多 job
├── tests/                          # 独立 CMake 工程（host 编译器）
│   ├── CMakeLists.txt              # project(rmec_tests) + FetchContent(googletest)
│   ├── support/
│   │   ├── fake_hal/               # 假 CubeMX/HAL 头（同名覆盖真头）
│   │   └── fake_sal/               # 假 sal_*.cc（链接期替换真实现）
│   ├── unit/                       # L1：纯逻辑用例，不依赖 fake
│   └── flow/                       # L3：真 module/app 源 + fake_sal
```

三条设计原则：

1. **目标代码零改动。** host 工程靠 include 路径优先级让 `Core/` 源文件解析到 `tests/support/fake_hal/` 的同名头。`sal_*.h` 用真实头，`sal_*.cc` 链接 fake 实现。除第 7 节声明的板型宏守卫（约 3 行）外，测试基建不产生针对 `Core/` 的 diff。
2. **被测源文件显式列表。** `tests/CMakeLists.txt` 按测试目标逐个列出参与编译的 `Core/` 文件，不用 glob。哪些代码"host 可编译"成为显式契约，纯逻辑文件引入 HAL 依赖会立刻编译失败。
3. **L3 不启动 FreeRTOS。** 假 `cmsis_os.h` 与假 `TaskManager` 只记录注册的 `init_func` 和 `task_func`。测试代码手动 `RunInit()` 加 `Step(n)` 步进，时序完全确定。

## 5. fake 层设计

### 5.1 fake_hal

只定义 `Core/` 实际用到的最小类型集：

- `UART_HandleTypeDef`、`CAN_HandleTypeDef`、`SPI_HandleTypeDef` 等结构体保留代码摸到的字段（如 `handle->Instance`）
- HAL 函数实现为向全局 `HalCallLog` 记录调用，返回可预设的状态码
- `CAN1`、`USART1` 等外设基址宏替换为假地址常量
- `cmsis_os.h` 提供 `osThreadCreate` 假实现：登记线程入口到 `TaskRegistry`，不真正调度

### 5.2 fake_sal

编译真实 `sal_*.h`，链接 `tests/support/fake_sal/` 的实现。构造函数照常注册实例与回调，契约与目标机一致。测试侧 API：

```cpp
namespace test {
  // CAN：断言发送、注入接收
  std::vector<CanFrame>& SentFrames(CAN_HandleTypeDef* bus);
  void InjectCanRx(CAN_HandleTypeDef* bus, uint16_t std_id, const uint8_t (&data)[8]);
  // UART：同上
  std::vector<UartPacket>& SentPackets(UART_HandleTypeDef* uart);
  void InjectUartRx(UART_HandleTypeDef* uart, const uint8_t* data, uint16_t len);
  // 时间：DwtInstance 假实现读可控时钟
  void AdvanceTimeMs(float ms);
  // 每个 TEST 之间清空记录与实例表
  void ResetAllFakes();
}
```

## 6. 首批用例

### 6.1 L1 回归用例（对应已修复 bug，防止回退）

| 用例 | 覆盖缺陷 |
|------|---------|
| `test_loop_queue` | 空队列 `popout`/`pop` 下溢、满队列 push 拒绝 |
| `test_pid` | `dt=0` 不产生 NaN/Inf、积分限幅 |
| `test_float2str` | `1.05f` 零填充、负数、缓冲边界 |
| `test_dwt_math` | 溢出跨度 `2^32` 而非 `UINT32_MAX` |
| `test_dt7_protocol` | 非法拨杆值 0、摇杆越界归零 |
| `test_referee_parser` | 合法帧、坏 CRC、超长帧拒收、断帧重同步 |
| `test_crc` | referee 与 vision 两套 CRC 已知向量 |
| `test_mit_codec` | 边界值、`span=0` 防除零 |
| `test_power_limiter` | 8 电机上限、`dt` 保护 |

### 6.2 L3 数据流用例

| 用例二进制 | 覆盖场景 |
|-----------|---------|
| `test_robot_task_modes` | 拨杆三位 × 模式迁移全路径、70 Hz 发布 + 200 Hz 消费不抖动、遥控离线后电机 Disable |
| `test_shoot_loader` | SINGLE/TRIPLE 边沿只走一发、REVERSE、热量保护停转 |
| `test_topic_wiring` | 每个 cmd topic 发布后步进消费任务，断言 fake CAN 收到非零输出（拦截"断头链路"） |

**约束：** `robot_topics.hpp` 的 Topic 是全局对象，`Subscribe()` 有 `MaxSubs=8` 上限且无反注册。因此 flow 测试按"每个任务簇一个测试二进制"组织，每个二进制只执行一次任务 init，用例之间靠发布新数据驱动状态迁移。这避免为测试给 `Topic` 加 reset 后门，坚持目标代码零改动。

**基线依赖：** L3 部分用例在 P0 修复合入前会失败。测试分支从 P0 分支合入后的 main 拉出，首批用例即全绿。

## 7. CI 工作流设计（L0）

单一 `ci.yml`，push 到任意分支与 PR 都触发，4 个 job 全部为 required check：

| Job | 内容 | 关键点 |
|-----|------|--------|
| `build-firmware` | 三板型矩阵交叉编译：`ONE_BOARD`、`GIMBAL_BOARD`、`CHASSIS_BOARD` | arm-none-eabi-gcc 用 pinned 版本（arm 官方 release 缓存安装），`-Wall -Werror` 生效，产出 elf/hex/bin artifact |
| `host-test` | 配置并运行 `tests/` 工程，`ctest --output-on-failure` | Ubuntu 自带 gcc，GoogleTest FetchContent 有缓存 |
| `clang-tidy` | 对 `Core/` 增量运行（PR 改动文件），基于交叉编译的 `compile_commands.json` | 首批只开 `bugprone-*`、`concurrency-*`、`performance-*`，`clang-analyzer` 后续再开 |
| `size-check` | 解析 `.map`，FLASH/RAM/CCMRAM 用量超过阈值（初始定为链接脚本容量的 90%）则失败 | 输出用量表到 job summary，便于观察趋势 |

矩阵编译依赖板型宏可从命令行注入。当前 `robot_def.hpp:3-17` 用 `#define` 写死板型，CI 需要 `-DBOARD_TYPE=xxx` 覆盖能力——这是本设计**唯一**需要动 `Core/` 的点：把板型宏改为 `#ifndef` 守卫（约 3 行），允许构建系统注入。此偏离在评审时明示。

## 8. 风险与缓解

| 风险 | 缓解 |
|------|------|
| fake_hal 头与真 HAL 结构漂移，测试通过但目标机行为不同 | fake 头只声明用到的字段。交叉编译 job 是同一 CI 的 required check，字段误用在目标机编译即失败 |
| 被测源文件显式列表维护成本 | 列表按测试目标分组，新增文件时编译报错会直接指出缺失项 |
| clang-tidy 存量告警阻塞日常合入 | 首批规则集从小起步，增量模式只查 PR 改动文件 |
| P0 分支合入时间不确定，L3 用例先红 | tests 分支基于合入后 main 拉出。若需提前开工，先交付 L0 + L1（不依赖 app 层修复） |
| GoogleTest FetchContent 依赖网络 | CI 缓存 `_deps`。若离线开发成为常态，切换 vendored（决策记录在案，切换成本一天内） |

## 9. 验收标准

1. `cmake -B build-tests -S tests && cmake --build build-tests && ctest --test-dir build-tests` 在干净 Ubuntu 环境一次通过
2. 6.1 全部 L1 用例存在且通过，人为回退任一已修复 bug（如删掉 PID 的 `dt` 保护）对应用例变红
3. 6.2 三个 flow 二进制通过，注释掉 `robot_task.cpp` 的任一 cmd 发布，`test_topic_wiring` 变红
4. CI 四个 job 在 PR 上全部运行并显示为 required，任一失败阻止合入
5. 三板型矩阵编译全绿，`size-check` 在 job summary 输出资源用量表
6. 全程 `git diff main -- Core/` 仅含第 7 节声明的板型宏守卫改动（约 3 行）

## 10. 后续工作

- P1 重构设计（并发原语、InstanceRegistry、错误码、IWDG）在本基建合入后启动，重构过程以 6.1/6.2 用例为回归网
- L2 模块级测试（BMI088 寄存器表仿真、电机驱动编解码）按需追加到 `tests/module/`
- 板上冒烟自检任务（栈水位、任务耗时上报）纳入 P1 设计讨论
