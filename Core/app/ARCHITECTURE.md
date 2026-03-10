# App 层软件架构设计

## 1. 总览

### 1.1 与 basic_framework 的对比

| 维度 | basic_framework (C) | powerful_framework (C++) |
|---|---|---|
| 通信 | `message_center` (string匹配, void*) | `Topic<T>` (编译期类型安全, SeqLock) |
| 任务 | 手动 `osThreadDef` + trampoline | `TaskManager` 封装 |
| 电机 | 全局数组 + 函数指针 | `Motor<Driver, Controller>` 模板组合 |
| 模式 | `typedef enum` (无作用域) | `enum class` (强类型) |
| 配置 | `robot_def.h` 包含 ins_task.h 等 | 分层: robot_def / robot_types / robot_topics |
| IMU | `ins_task.c` God Object | Bmi088(驱动) + Ins(算法) + ins_task(编排) |
| 多板 | `#ifdef ONE_BOARD` 条件编译 | 同, 但 Topic 可透明替换为 CAN 传输 |

### 1.2 设计原则

```
SRP  — 每个 app 文件只做一件事 (CMD=指挥, Chassis=运动学, Gimbal=云台控制)
DIP  — 子系统不依赖彼此, 只依赖 Topic<T> 数据结构
ISP  — robot_types.hpp 零 heavy 依赖, 任何 app 都能 include 而不引入 ARM DSP
OCP  — 换底盘类型只改 chassis.cpp, 不动 CMD/Gimbal/Shoot
```

## 2. 文件结构

```
Core/app/
├── robot_def.hpp           # 板型宏 + 机械常量 (纯 #define, 零 include)
├── robot_types.hpp         # enum class 模式 + 命令/反馈结构体
├── robot_topics.hpp        # 所有 inline Topic<T> 实例 (pub/sub 枢纽)
├── robot.cpp               # RobotInit() + 编译验证 static_assert
│
├── ins_task.hpp/.cpp       # [已实现] 1 kHz IMU + EKF → Topic<InsData>
│
├── robot_task.hpp          # RobotTaskStart() 声明
├── robot_task.cpp          # 200 Hz 主任务: 顺序调用 CMD→Gimbal→Shoot→Chassis
│
├── cmd/
│   ├── robot_cmd.hpp       # RobotCmdInit(), RobotCmdTask()
│   └── robot_cmd.cpp       # 遥控器/键鼠输入 → 模式决策 → 发布命令
│
├── chassis/
│   ├── chassis.hpp         # ChassisInit(), ChassisTask()
│   └── chassis.cpp         # 麦轮运动学 + 功率限制
│
├── gimbal/
│   ├── gimbal.hpp          # GimbalInit(), GimbalTask()
│   └── gimbal.cpp          # Yaw/Pitch 双轴 IMU 闭环
│
└── shoot/
    ├── shoot.hpp           # ShootInit(), ShootTask()
    └── shoot.cpp           # 摩擦轮 + 拨弹盘
```

## 3. 分层依赖图

```
┌─────────────────────────────────────────────────────────┐
│                      App 层                              │
│                                                          │
│  robot_def.hpp ← robot_types.hpp ← robot_topics.hpp     │
│       (常量)        (枚举+结构体)      (Topic 实例)        │
│                         │                  │             │
│  ┌──────┐  ┌────────┐  │  ┌─────────┐  ┌──────┐        │
│  │ CMD  │  │Chassis │  │  │ Gimbal  │  │Shoot │        │
│  └──┬───┘  └───┬────┘  │  └────┬────┘  └──┬───┘        │
│     │          │        │       │           │            │
│     └──────────┴────────┴───────┴───────────┘            │
│              全部通过 Topic<T> 通信                        │
│              零 #include 其他 app 头文件                   │
├──────────────────────────────────────────────────────────┤
│                     Module 层                             │
│  Motor<D,C>   PID   Ins   Bmi088   PowerLimiter          │
├──────────────────────────────────────────────────────────┤
│                      SAL 层                               │
│  CAN  SPI  UART  GPIO  PWM  DWT  TaskManager  Topic      │
└──────────────────────────────────────────────────────────┘
```

**关键**: 没有任何一条从 Chassis→Gimbal 或 CMD→Chassis 的直接依赖。
所有 app 只依赖 `robot_types.hpp` + `robot_topics.hpp` + 自身需要的 Module 层头文件。

## 4. 启动流程

### 4.1 与 basic_framework 对比

| 阶段 | basic_framework (C) | powerful_framework (C++) |
|---|---|---|
| 外设初始化 | `main.c` → `MX_*_Init()` | 同 |
| C++ 入口 | `RobotInit()` → `BSPInit()` + `OSTaskInit()` | `RobotInit()` → `InsTaskStart()` + 未来任务 |
| 调度器启动 | `osKernelStart()` | 同 |
| `freertos.c` | 仅 USB init + terminate | 同 (不碰 CubeMX 生成文件) |

### 4.2 启动时序

```
main.c (C):
  HAL_Init()
  SystemClock_Config()
  MX_GPIO/DMA/CAN/SPI/TIM/USART_Init()
  DWTInit_C(168)
  RobotInit()                  ← C++ 侧唯一入口
  MX_FREERTOS_Init()           ← CubeMX 默认 (defaultTask → USB init → terminate)
  osKernelStart()              ← 调度器启动, 所有任务开始运行
```

```
robot.cpp (C++):
  extern "C" void RobotInit() {
      InsTaskStart();            // 创建 1 kHz INS 任务
      // RobotTaskStart();       // 未来: 200 Hz 主任务
      // DaemonTaskStart();      // 未来: 100 Hz 看门狗
  }
```

**设计原则**: `main.c` 和 `freertos.c` 保持 CubeMX 可重新生成状态, 只在 `USER CODE` 区域添加
`RobotInit()` 调用。所有 FreeRTOS 任务创建通过 `TaskManager` 在 C++ 侧完成。

## 5. 任务调度

| 任务 | 频率 | 优先级 | 栈 (words) | 职责 |
|---|---|---|---|---|
| `ins_task` | 1 kHz | Realtime | 512 | BMI088→EKF→InsData→Topic |
| `robot_task` | 200 Hz | Normal | 512 | CMD + Gimbal + Shoot + Chassis (顺序执行) |
| `daemon_task` | 100 Hz | Normal | 128 | 看门狗 + 蜂鸣器 + 离线检测 |
| `ui_task` | ~1 Hz | BelowNormal | 512 | 裁判系统 UI 绘制 |

### 为什么 robot_task 是 200 Hz 而不是 1 kHz?

- 遥控器输入本身只有 ~50 Hz, 200 Hz 已经绰绰有余
- 运动学解算 (麦轮/云台) 不需要 1 kHz — 设定目标即可
- 电机 PID 由 Motor::Update() 在 robot_task 中执行, 200 Hz 对大多数场景足够
- 如果需要 1 kHz PID (高性能云台), 可以后期将云台电机单独提到 ins_task 中

### 执行时序

```
每 5ms (robot_task):
  ┌─ RobotCmdTask()
  │   ├── 读取遥控器/键鼠/视觉
  │   ├── 模式决策
  │   └── 发布: chassis_cmd, gimbal_cmd, shoot_cmd
  │
  ├─ GimbalTask()
  │   ├── 订阅: gimbal_cmd, ins_topic
  │   ├── 模式切换 (零力/自由/陀螺仪)
  │   ├── 电机 Update() (PID + 输出)
  │   └── 发布: gimbal_feed
  │
  ├─ ShootTask()
  │   ├── 订阅: shoot_cmd
  │   ├── 摩擦轮/拨弹盘控制
  │   ├── 电机 Update()
  │   └── 发布: shoot_feed
  │
  ├─ ChassisTask()
  │   ├── 订阅: chassis_cmd
  │   ├── 麦轮运动学
  │   ├── 功率限制
  │   ├── 电机 Update()
  │   └── 发布: chassis_feed
  │
  └─ DjiDriver::FlushAll()   // 一次性发送所有 CAN 帧

每 1ms (ins_task):
  BMI088 读取 → Ins 更新 → Topic 发布 → 温控
```

## 6. Topic 通信拓扑

### 6.1 Topic 定义 (robot_topics.hpp)

```cpp
#pragma once
#include "topic.hpp"
#include "ins_data.hpp"
#include "robot_types.hpp"

// IMU 姿态 (1 kHz 发布)
inline Topic<InsData>          ins_topic;

// CMD → 子系统 命令 (200 Hz 发布)
inline Topic<ChassisCmdData>   chassis_cmd_topic;
inline Topic<GimbalCmdData>    gimbal_cmd_topic;
inline Topic<ShootCmdData>     shoot_cmd_topic;

// 子系统 → CMD 反馈 (200 Hz 发布)
inline Topic<ChassisFeedData>  chassis_feed_topic;
inline Topic<GimbalFeedData>   gimbal_feed_topic;
inline Topic<ShootFeedData>    shoot_feed_topic;
```

### 6.2 数据流

```
RC/键鼠                                           裁判系统
   │                                                  │
   ▼                                                  ▼
┌──────┐  chassis_cmd  ┌─────────┐  chassis_feed  ┌──────┐
│      │──────────────→│ Chassis │───────────────→│      │
│      │  gimbal_cmd   ├─────────┤  gimbal_feed   │      │
│ CMD  │──────────────→│ Gimbal  │───────────────→│ CMD  │
│      │  shoot_cmd    ├─────────┤  shoot_feed    │      │
│      │──────────────→│  Shoot  │───────────────→│      │
└──────┘               └─────────┘                └──────┘
                            ▲
                            │ ins_topic (1 kHz)
                       ┌────┴────┐
                       │INS Task │
                       └─────────┘
```

### 6.3 订阅矩阵

| Topic | 发布者 | 订阅者 |
|---|---|---|
| `ins_topic` | ins_task | gimbal, (cmd 可选) |
| `chassis_cmd_topic` | cmd | chassis |
| `chassis_feed_topic` | chassis | cmd |
| `gimbal_cmd_topic` | cmd | gimbal |
| `gimbal_feed_topic` | gimbal | cmd |
| `shoot_cmd_topic` | cmd | shoot |
| `shoot_feed_topic` | shoot | cmd |

## 7. 数据类型设计 (robot_types.hpp)

### 7.1 模式枚举

```cpp
enum class RobotStatus : uint8_t { STOP, READY };

enum class ChassisMode : uint8_t {
    ZERO_FORCE,              // 零力矩, 电机不输出
    FOLLOW_GIMBAL_YAW,       // 底盘跟随云台 yaw
    NO_FOLLOW,               // 底盘与云台解耦
    ROTATE,                  // 小陀螺旋转
};

enum class GimbalMode : uint8_t {
    ZERO_FORCE,              // 零力矩
    FREE_MODE,               // 编码器反馈 (自由模式)
    GYRO_MODE,               // IMU 反馈 (陀螺仪模式)
};

enum class ShootMode : uint8_t { OFF, ON };
enum class FrictionMode : uint8_t { OFF, ON };
enum class LidMode : uint8_t { OPEN, CLOSE };
enum class LoaderMode : uint8_t {
    STOP,
    REVERSE,                 // 反转 (卡弹)
    SINGLE,                  // 单发
    TRIPLE,                  // 三连发
    BURST,                   // 连发
};
enum class BulletSpeed : uint8_t { SMALL_15, SMALL_18, SMALL_30 };
```

### 7.2 命令结构体 (CMD → 子系统)

```cpp
struct ChassisCmdData {
    float vx           = 0;     // 前进速度 (云台坐标系)
    float vy           = 0;     // 横移速度 (云台坐标系)
    float wz           = 0;     // 旋转角速度
    float offset_angle = 0;     // 云台-底盘 yaw 偏差 (度)
    ChassisMode mode   = ChassisMode::ZERO_FORCE;
    int speed_buff     = 100;   // 速度百分比
};

struct GimbalCmdData {
    float yaw          = 0;     // 目标 yaw (绝对角度, 度)
    float pitch        = 0;     // 目标 pitch (绝对角度, 度)
    GimbalMode mode    = GimbalMode::ZERO_FORCE;
};

struct ShootCmdData {
    ShootMode    shoot_mode    = ShootMode::OFF;
    LoaderMode   load_mode     = LoaderMode::STOP;
    FrictionMode friction_mode = FrictionMode::OFF;
    LidMode      lid_mode      = LidMode::CLOSE;
    BulletSpeed  bullet_speed  = BulletSpeed::SMALL_15;
    float        shoot_rate    = 0;     // 连发频率 (发/秒)
    uint8_t      rest_heat     = 0;     // 剩余热量
};
```

### 7.3 反馈结构体 (子系统 → CMD)

```cpp
struct ChassisFeedData {
    uint8_t     rest_heat    = 0;
    BulletSpeed bullet_speed = BulletSpeed::SMALL_15;
    // 裁判系统数据由底盘板透传
};

struct GimbalFeedData {
    float yaw_total         = 0;    // 多圈 yaw (IMU)
    float yaw_single_round  = 0;    // 单圈 yaw (编码器, 用于 offset 计算)
    float pitch             = 0;
};

struct ShootFeedData {
    // 预留
};
```

所有结构体必须满足 `std::is_trivially_copyable_v<T>` (Topic 要求)。

## 8. 各子系统设计

### 8.1 CMD (robot_cmd.cpp)

**职责**: 唯一的模式决策者。读取遥控器/键鼠/视觉输入, 生成控制指令。

**持有资源**:
- RC 遥控器实例 (UART3)
- 视觉通信实例 (UART1 / VCP)
- 7 个 Topic 的 publisher/subscriber

**状态机**:
```
RC 右拨杆:
  上 → ROBOT_READY, SHOOT_ON
  中 → CHASSIS_NO_FOLLOW + GIMBAL_FREE_MODE
  下 → CHASSIS_ROTATE + GIMBAL_GYRO_MODE

RC 左拨杆:
  上 → 键鼠模式 (MouseKeySet)
  中 → 视觉模式 (未来扩展)
  下 → 遥控器模式 (RemoteControlSet)

急停: 拨轮 > 300 或 ROBOT_STOP → 全部 ZERO_FORCE
```

**`CalcOffsetAngle()`**: 从 GimbalFeedData 的编码器值计算底盘-云台 yaw 偏差,
处理 0-360° 跨越, 写入 `chassis_cmd.offset_angle`。

### 8.2 Chassis (chassis.cpp)

**持有资源**:
- 4 × `Motor<DjiDriver, CascadePid>` (M3508, hcan1, id 1-4)
- `PowerLimiter` 实例
- `TopicReader<ChassisCmdData>*`

**控制流**:
```
1. 读取 chassis_cmd
2. 模式分发:
   ZERO_FORCE           → 停止所有电机
   FOLLOW_GIMBAL_YAW    → wz = -Kp * offset * |offset| (非线性跟随)
   NO_FOLLOW            → wz = 0
   ROTATE               → wz = 常数 (小陀螺)
3. 坐标变换: 云台系 (vx,vy) → 底盘系 (用 offset_angle 旋转)
4. 麦轮逆运动学 → 4 路轮速
5. 功率限制 (PowerLimiter)
6. 电机 Update()
7. 发布 chassis_feed
```

**麦轮运动学** (与 basic_framework 一致):
```
                 前
           ┌───────────┐
     LF ○──┤           ├──○ RF
           │   Gimbal  │
     LB ○──┤   Center  ├──○ RB
           └───────────┘

vt_lf = -vx - vy - wz * d_lf
vt_rf = -vx + vy - wz * d_rf
vt_lb =  vx - vy - wz * d_lb
vt_rb =  vx + vy - wz * d_rb

d = sqrt((WHEEL_BASE/2 ± OFFSET_X)² + (TRACK_WIDTH/2 ± OFFSET_Y)²)
```

### 8.3 Gimbal (gimbal.cpp)

**持有资源**:
- 2 × `Motor<DjiDriver, CascadePid>` (GM6020, yaw on hcan1, pitch on hcan2)
- `TopicReader<GimbalCmdData>*`
- `TopicReader<InsData>*`
- 本地 `InsData` 缓存 (用于 IMU 反馈指针)

**IMU 反馈注入** (关键设计):
```cpp
// GimbalInit() 中:
static InsData gimbal_ins;   // 文件作用域, 供电机外部反馈指针使用

// GimbalTask() 中:
ins_reader->Read(gimbal_ins);                        // 每周期从 Topic 读取
yaw_motor.SetExternalFeedback(&gimbal_ins.yaw_total, // 角度
                              &gimbal_ins.euler[2]);  // 角速度 (待确认轴向)
```

**模式**:
```
ZERO_FORCE  → 停止 yaw/pitch 电机
FREE_MODE   → 使用编码器反馈 (MOTOR_FEED)
GYRO_MODE   → 使用 IMU 反馈 (OTHER_FEED), yaw 用 YawTotal, pitch 用 Pitch
```

### 8.4 Shoot (shoot.cpp)

**持有资源**:
- 2 × `Motor<DjiDriver, CascadePid>` (M3508 摩擦轮, hcan2, id 1-2)
- 1 × `Motor<DjiDriver, CascadePid>` (M2006 拨弹盘, hcan2, id 3)
- `TopicReader<ShootCmdData>*`

**摩擦轮控制**:
```
FRICTION_OFF → 两轮 ref = 0
FRICTION_ON  → 根据 bullet_speed 设定转速:
  SMALL_15: ±xxx rpm (需实测标定)
  SMALL_18: ±xxx rpm
  SMALL_30: ±xxx rpm
注意: 左右轮方向相反
```

**拨弹盘控制**:
```
LOAD_STOP    → 速度环, ref = 0
LOAD_SINGLE  → 角度环, ref += ONE_BULLET_DELTA_ANGLE (36°)
LOAD_TRIPLE  → 角度环, ref += 3 × 36°
LOAD_BURST   → 速度环, ref = shoot_rate × 360 × REDUCTION_RATIO / 8
LOAD_REVERSE → 速度环, ref = -xxx (卡弹反转)
```

## 9. 多板支持

### 9.1 板型定义 (robot_def.hpp)

```cpp
// 三选一, 互斥
#define ONE_BOARD        // 单板控制整车
// #define CHASSIS_BOARD // 底盘板 (底盘 + 裁判系统)
// #define GIMBAL_BOARD  // 云台板 (CMD + 云台 + 发射)
```

### 9.2 条件编译模式

```cpp
// robot_task.cpp init_func:
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCmdInit();
    GimbalInit();
    ShootInit();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif
```

### 9.3 双板 CAN 通信 (未来扩展)

单板模式下, CMD→Chassis 使用 `Topic<ChassisCmdData>` 内存通信。
双板模式下, GIMBAL_BOARD 的 CMD 通过 CAN 发送 `ChassisCmdData` 到 CHASSIS_BOARD:

```
GIMBAL_BOARD:                        CHASSIS_BOARD:
  CMD → CAN TX (0x312)       →       CAN RX → Chassis
  CMD ← CAN RX (0x311)       ←       CAN TX ← Chassis
```

实现方式: 在 `robot_cmd.cpp` 中用 `#ifdef GIMBAL_BOARD` 替换 Topic 发布为 CAN 发送。
CAN 传输层封装为统一接口, 使切换对子系统透明。

## 10. robot_def.hpp 常量

```cpp
#pragma once

// ======================== 板型 ========================
#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD

#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD))  || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error "Board type conflict! Define exactly one."
#endif

// ======================== 云台 ========================
inline constexpr uint16_t YAW_CHASSIS_ALIGN_ECD   = 2711;  // TODO: 实测标定
inline constexpr bool     YAW_ECD_GREATER_THAN_4096 = false;
inline constexpr uint16_t PITCH_HORIZON_ECD        = 3412;  // TODO: 实测标定
inline constexpr float    PITCH_MAX_ANGLE          = 25.0f; // TODO: 实测
inline constexpr float    PITCH_MIN_ANGLE          = -20.0f;

// ======================== 底盘 ========================
inline constexpr float WHEEL_BASE              = 350.0f;  // mm, 前后轮距
inline constexpr float TRACK_WIDTH             = 300.0f;  // mm, 左右轮距
inline constexpr float CENTER_GIMBAL_OFFSET_X  = 0.0f;    // mm, 云台中心偏移
inline constexpr float CENTER_GIMBAL_OFFSET_Y  = 0.0f;
inline constexpr float RADIUS_WHEEL            = 60.0f;   // mm
inline constexpr float REDUCTION_RATIO_WHEEL   = 19.0f;

// ======================== 发射 ========================
inline constexpr float ONE_BULLET_DELTA_ANGLE  = 36.0f;   // 度, 单发角度增量
inline constexpr float REDUCTION_RATIO_LOADER  = 36.0f;

// ======================== IMU 安装方向 ========================
inline constexpr int8_t GYRO2GIMBAL_DIR_YAW    = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_PITCH  = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_ROLL   = 1;
```

## 11. 实现路线图

### Phase 1: 基础框架 (当前可做)
1. `robot_def.hpp` — 板型宏 + 机械常量
2. `robot_types.hpp` — 模式枚举 + 命令/反馈结构体
3. 扩展 `robot_topics.hpp` — 添加 chassis/gimbal/shoot 的 7 个 Topic
4. `robot_task.hpp/.cpp` — 200 Hz 主任务框架 (空 Init/Task 桩函数)

### Phase 2: CMD + 底盘
5. `cmd/robot_cmd.cpp` — 遥控器初始化 + 输入映射
6. `chassis/chassis.cpp` — 4 × M3508 + 麦轮运动学

### Phase 3: 云台 + 发射
7. `gimbal/gimbal.cpp` — 2 × GM6020 + IMU 反馈
8. `shoot/shoot.cpp` — 摩擦轮 + 拨弹盘

### Phase 4: 外围
9. 功率限制集成 (PowerLimiter + SuperCap)
10. 裁判系统 UI
11. 视觉通信
12. 看门狗 / 离线检测

### Phase 5: 多板
13. CAN 通信层封装
14. 双板条件编译测试

## 12. 关键设计模式总结

| 模式 | 说明 |
|---|---|
| **Pub-Sub 解耦** | 子系统间零 include, 全部通过 Topic<T> 通信 |
| **CMD 单点决策** | 只有 CMD 写入 mode 枚举, 子系统只读 |
| **配置分层** | robot_def (常量) → robot_types (类型) → robot_topics (实例) |
| **TaskManager 编排** | 每个独立任务用 TaskManager lambda 封装, 生命周期清晰 |
| **文件作用域 static** | 每个 app 的状态都是 static, 等价于单例但无 class 开销 |
| **Motor 模板零虚函数** | `Motor<DjiDriver, CascadePid>` 编译期组合, 零运行时开销 |
| **ISP 防火墙** | ins_data.hpp / robot_types.hpp 是轻量头文件, 不传递 heavy 依赖 |
