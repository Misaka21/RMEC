# App 层软件架构设计

## 1. 总览

### 1.1 与 basic_framework 的对比

| 维度 | basic_framework (C) | powerful_framework (C++) |
|---|---|---|
| 通信 | `message_center` (string匹配, void*) | `Topic<T>` (编译期类型安全, SeqLock) |
| 任务 | 手动 `osThreadDef` + trampoline | `TaskManager` 封装 |
| 电机 | 全局数组 + 函数指针 | `Motor<Driver, Controller>` 模板组合 |
| 模式 | `typedef enum` (无作用域) | `enum class` (强类型) |
| 配置 | `robot_def.h` 包含 ins_task.h 等 | 分层: robot_def (常量) / robot_topics (类型+Topic) |
| IMU | `ins_task.c` God Object | Bmi088(驱动) + Ins(算法) + ins_task(编排) |
| 多板 | `#ifdef ONE_BOARD` 条件编译 | 同, 但 Topic 可透明替换为 CAN 传输 |

### 1.2 设计原则

```
SRP  — 每个 app 文件只做一件事 (CMD=指挥, Chassis=运动学, Gimbal=云台控制)
DIP  — 子系统不依赖彼此, 只依赖 Topic<T> 数据结构
ISP  — robot_topics.hpp / ins_data.hpp 零 heavy 依赖, 任何 app 都能 include 而不引入 ARM DSP
OCP  — 换底盘类型只改 chassis.cpp, 不动 CMD/Gimbal/Shoot
```

## 2. 文件结构

### 2.1 basic_framework 的 task 分析

basic_framework 的 task 设计本身很好 (职责单一, 频率合理), 但 **task 放在了 module 层, 这是不对的**:

| task 函数 | basic_framework 位置 | 问题 |
|---|---|---|
| `INS_Init/Task` | `modules/imu/ins_task.c` | 引用 `robot_def.h` 硬件引脚, 不可复用 |
| `MotorControlTask` | `modules/motor/motor_task.c` | 遍历全局电机数组, 隐含 app 知识 |
| `DaemonTask` | `modules/daemon/daemon.c` | 组件本身可复用, 但 task 循环不应在 module |
| `BuzzerTask` | `modules/alarm/buzzer.c` | 同上 |
| `UITask` | `modules/referee/referee_task.c` | 引用 `robot_def.h` 中的模式枚举 |
| `VisionSend` | `modules/master_machine/` | 引用 `robot_def.h` 中的配置 |

**问题本质**: 这些 task 函数引用了 `robot_def.h` 的硬件配置和机器人参数,
换车时需要修改, 违反了 "module 层换车不改" 的原则。

**powerful_framework 的改进**: Task 编排全部放 app 层, module 层只提供纯组件类。

### 2.2 powerful_framework 的文件结构

```
Core/module/                        ← 模块层 (纯组件, 换车不改)
│
│   不创建 FreeRTOS 任务
│   不引用 robot_def.hpp
│   只提供 class + Init()/Update()/Tick() 方法
│
├── algorithm/
│   ├── pid_controller.hpp/.cpp     # [已实现] PID 控制器
│   ├── ahrs_math.hpp               # [已实现] 向量/坐标变换
│   ├── quaternion_ekf.hpp/.cpp     # [已实现] 四元数 EKF
│   └── power_limiter.hpp/.cpp      # [已实现] 功率限制 (RLS)
│
├── imu/
│   ├── bmi088_reg.hpp              # [已实现] 寄存器定义
│   ├── bmi088.hpp/.cpp             # [已实现] BMI088 驱动 (接收 Bmi088Config)
│   ├── ins_data.hpp                # [已实现] InsData 轻量结构体
│   └── ins.hpp                     # [已实现] Ins 姿态解算
│
├── motor/
│   ├── motor.hpp                   # [已实现] Motor<D,C> 模板
│   ├── motor_measure.hpp           # [已实现] 编码器多圈计数
│   ├── driver/dji_driver.hpp/.cpp  # [已实现] DJI 电机驱动
│   ├── controller/cascade_pid.hpp  # [已实现] 串级 PID
│   └── controller/mit_passthrough.hpp # [已实现] MIT 直通
│
├── remote/                         # [已实现] Remote<Protocol> 遥控器模板
│   ├── remote.hpp                  # Remote<Protocol> + PublishFn 回调
│   └── protocol/
│       ├── dt7_data.hpp            # Dt7Data 轻量数据结构 (供 Topic)
│       └── dt7_protocol.hpp/.cpp   # DT7/DR16 DBUS 18 字节帧解析
│
├── referee/                        # 裁判系统协议解析 + UI 绘制 API
│   └── referee.hpp/.cpp
│
├── daemon/                         # [已实现] DaemonInstance 健康监测 (FSM + 时间戳)
│   └── daemon.hpp                  # header-only, UNSEEN/ONLINE/OFFLINE 状态机
│
├── alarm/                          # 蜂鸣器 (优先级报警, Tick() 扫描)
│   └── buzzer.hpp/.cpp
│
├── vision/                         # 视觉协议 (编解码, Send/Recv)
│   └── vision.hpp/.cpp
│
├── super_cap/                      # 超级电容通信
│   └── super_cap.hpp/.cpp
│
├── topic.hpp                       # [已实现] SeqLock 发布订阅
└── general_def.hpp                 # [已实现] 数学常量 + 工具函数

Core/app/                           ← 应用层 (编排 + 机器人逻辑)
│
│   *_task/ 文件夹一般对应独立 FreeRTOS 线程 (remote_task 例外, 仅做初始化+回调编排)
│   robot_task 只做指令决策/发布, motor_task 统一执行电机闭环与 CAN 发送
│
├── robot_def.hpp                   # 板型宏 + 硬件引脚 + 机械常量
├── robot_topics.hpp                # enum class + 命令/反馈结构体 + Topic 实例
├── robot.cpp                       # RobotInit() 入口 (启动所有 *_task)
│
├── ins_task/                       # 独立 TaskManager, 1 kHz, Realtime
│   ├── ins_task.hpp                # [已实现]
│   └── ins_task.cpp                # 硬件配置 → Bmi088 → Ins → Topic → 温控
│
├── remote_task/                    # [已实现] 遥控器初始化 + ISR 发布编排 (非 FreeRTOS 任务)
│   ├── remote_task.hpp             # RemoteInit() + RemoteIsOnline() + GetRemote()
│   └── remote_task.cpp             # 创建 Remote + Daemon, 注入 publish 回调 → remote_topic
│
├── robot_task/                     # 独立 TaskManager, 200 Hz, Normal
│   ├── robot_task.hpp
│   ├── robot_task.cpp              # 顺序调用 cmd + 发布各子系统目标(不直接驱动电机)
│   └── cmd.hpp / cmd.cpp           # 遥控器/键鼠 → 模式决策 → 发布命令
│
├── motor_task/                     # 独立 TaskManager, 1 kHz, Normal
│   ├── motor_task.hpp
│   ├── motor_task.cpp              # 统一执行电机 PID + DjiDriver::FlushAll()
│   ├── chassis.hpp / chassis.cpp   # 麦轮运动学 + 功率限制
│   ├── gimbal.hpp / gimbal.cpp     # Yaw/Pitch IMU 闭环
│   └── shoot.hpp / shoot.cpp       # 摩擦轮 + 拨弹盘
│
├── daemon_task/                    # [已实现] 独立 TaskManager, 100 Hz, Normal
│   ├── daemon_task.hpp
│   └── daemon_task.cpp             # DaemonInstance::TickAll()
│
└── ui_task/                        # 独立 TaskManager, ~1 Hz, BelowNormal
    ├── ui_task.hpp
    └── ui_task.cpp                 # 裁判系统 UI 绘制
```

### 2.3 职责边界

| | 模块层 (`Core/module/`) | 应用层 (`Core/app/`) |
|---|---|---|
| **提供** | 纯组件类: `Bmi088`, `Ins`, `Daemon`, `Buzzer`, `Remote` 等 | Task 编排 + 机器人逻辑 |
| **接口** | `Init(config)`, `Update()`, `Tick()`, `Data()` | `TaskManager` lambda 调用 module 方法 |
| **依赖** | 只依赖 SAL 层 + 其他 module | 依赖 module + `robot_def.hpp` |
| **FreeRTOS** | **不创建任务**, 不调用 `osDelay` | 创建 TaskManager, 控制频率和优先级 |
| **robot_def** | **不引用** | 引用, 填入硬件配置 |
| **换车** | **不改** | 改引脚、参数、子系统逻辑 |

## 3. 分层依赖图

```
┌──────────────────────────────────────────────────────────────────┐
│                      App 层 (编排 + 机器人逻辑)                    │
│                                                                   │
│  robot_def.hpp ←─── robot_topics.hpp                              │
│    (硬件常量)        (枚举+结构体+Topic)                            │
│                                                                   │
│  ins_task/ (1kHz)   robot_task/ (200Hz)   motor_task/ (1kHz)      │
│  daemon_task/ (100Hz)   ui_task/ (~1Hz)                           │
│                                                                   │
│  robot_task/                                                       │
│    └── cmd ──→ 发布命令 Topic                                      │
│                                                                   │
│  motor_task/                                                       │
│    ├── chassis ──→ 麦轮运动学                                      │
│    ├── gimbal  ──→ 结合 ins_topic 的 1kHz 闭环                    │
│    ├── shoot   ──→ 摩擦轮/拨弹                                     │
│    └── FlushAll()  (单写者, 唯一发送出口)                          │
│                                                                   │
│  所有子系统通过 Topic<T> 通信, 零直接依赖                            │
├──────────────────────────────────────────────────────────────────┤
│                Module 层 (纯组件, 无 task, 无 robot_def)           │
│                                                                   │
│  算法:   PID  QuaternionEkf  Ins  ahrs_math  PowerLimiter        │
│  驱动:   Bmi088  Motor<D,C>  DjiDriver                           │
│  通信:   Remote  Referee  Vision  SuperCap                        │
│  服务:   Daemon  Buzzer                                           │
│  基础:   Topic<T>  general_def                                    │
├──────────────────────────────────────────────────────────────────┤
│                         SAL 层                                    │
│  CAN  SPI  UART  GPIO  PWM  DWT  TaskManager  Flash  Log  USB   │
└──────────────────────────────────────────────────────────────────┘
```

## 4. 启动流程

### 4.1 与 basic_framework 对比

| 阶段 | basic_framework (C) | powerful_framework (C++) |
|---|---|---|
| 外设初始化 | `main.c` → `MX_*_Init()` | 同 |
| C++ 入口 | `RobotInit()` → `BSPInit()` + `OSTaskInit()` | `RobotInit()` → `InsTaskStart()` + `MotorTaskStart()` + 未来任务 |
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
      RemoteInit();              // 创建遥控器实例 + Daemon, 启动 UART DMA, 注入 publish 回调
      MotorTaskStart();          // 创建 1 kHz 电机闭环任务 (Subscribe + Init pre-scheduler)
      DaemonTaskStart();         // 启动 100 Hz 看门狗任务
      // RobotTaskStart();       // 未来: 200 Hz 指令任务
  }
```

**设计原则**: `main.c` 和 `freertos.c` 保持 CubeMX 可重新生成状态, 只在 `USER CODE` 区域添加
`RobotInit()` 调用。所有 FreeRTOS 任务创建通过 `TaskManager` 在 C++ 侧完成。

## 5. 任务调度

### 5.1 与 basic_framework 任务对比

basic_framework 有 5 个独立 FreeRTOS 任务:

| basic_framework 任务 | 频率 | 内容 | powerful_framework 对应 |
|---|---|---|---|
| `ins_task` (1 kHz) | AboveNormal | BMI088 + EKF + VisionSend() | `ins_task` (已实现) |
| `motor_task` (1 kHz) | Normal | 所有电机 PID 计算 + CAN 发送 | `motor_task` (单写者) |
| `robot_task` (200 Hz) | Normal | CMD 设定目标 + Chassis/Gimbal/Shoot 逻辑 | `robot_task` |
| `daemon_task` (100 Hz) | Normal | 离线检测 (Daemon) + BuzzerTask() | `daemon_task` |
| `ui_task` (~1 Hz) | Normal | 裁判系统 UI 绘制 | `ui_task` |

### 5.2 为什么需要单独 motor_task?

basic_framework 的设计:
- `robot_task` (200 Hz): 设定电机目标值 (setpoint)
- `motor_task` (1 kHz): 遍历全局电机数组, 计算 PID, 发送 CAN

这依赖于**全局电机注册表** (`DJIMotorInstance` 静态数组), 所有电机集中管理。

powerful_framework 当前实现中, `DjiDriver` 使用静态共享发送缓冲区（`SetOutput()` 写缓冲，`FlushAll()` 发送并清空）。
若 `ins_task`、`robot_task` 同时调用 `Motor::Update()/SetOutput()/FlushAll()`，会出现跨任务竞争和频率失真。

因此采用 **single-writer** 设计：
- `robot_task` / `ins_task` 只负责发布目标值（Topic）
- `motor_task` 是唯一允许调用 `Motor::Update()/SetOutput()/FlushAll()` 的任务
- CAN 发送出口只有一个，避免并发写共享发送缓冲

具体策略:

| 电机 | 所在任务 | PID 频率 | 说明 |
|---|---|---|---|
| 底盘 M3508 ×4 | motor_task | 1 kHz | 速度环 + 功率限制 |
| 摩擦轮 M3508 ×2 | motor_task | 1 kHz | 速度环 |
| 拨弹盘 M2006 ×1 | motor_task | 1 kHz | 角度/速度环 |
| 云台 GM6020 ×2 | motor_task | 1 kHz | IMU 反馈闭环 |

> 所有电机统一 1 kHz, 与 basic_framework 一致。CAN 单写者避免跨任务竞争。

### 5.3 任务总表

| 任务 | 频率 | 优先级 | 栈 (words) | 职责 |
|---|---|---|---|---|
| `ins_task` | 1 kHz | Realtime | 512 | BMI088→EKF→InsData→Topic→温控 |
| `robot_task` | 200 Hz | Normal | 512 | CMD + 模式决策 + 发布各子系统目标 Topic |
| `motor_task` | 1 kHz | Normal | 512 | 统一电机闭环 (全 1 kHz) + FlushAll() |
| `daemon_task` | 100 Hz | Normal | 128 | 离线检测 + 蜂鸣器 |
| `ui_task` | ~1 Hz | BelowNormal | 512 | 裁判系统 UI 绘制 |

### 5.4 执行时序

```
每 5ms (robot_task):
  ┌─ RobotCmdTask()
  │   ├── 读取 remote_topic (遥控器) / 视觉
  │   ├── rc->Tick() 离线检测
  │   ├── 模式决策
  │   └── 发布: chassis_cmd, gimbal_cmd, shoot_cmd

每 1ms (ins_task):
  └─ BMI088 读取 → Ins 更新 → Topic<InsData> 发布 → 温控(500 Hz)

每 1ms (motor_task):
  ┌─ 订阅: chassis_cmd / gimbal_cmd / shoot_cmd / ins_topic
  │
  ├─ GimbalTask()             // 1 kHz, IMU 反馈闭环 + 发布 gimbal_feed
  ├─ ChassisTask()            // 1 kHz, 麦轮分解 + 功率限制
  ├─ ShootTask()              // 1 kHz, 摩擦轮/拨弹盘模式处理
  │
  └─ FlushAll()               // DJI/DM/HT/LK 统一 CAN 发送 (单写者)

每 10ms (daemon_task):
  ├─ DaemonInstance::TickAll()   // FSM 状态检查, 边沿触发回调
  └─ BuzzerTask()             // 按优先级扫描蜂鸣器报警

~1s (ui_task):
  └─ UiTask()                 // 裁判系统 UI 绘制 + 发包
```

## 6. Topic 通信拓扑

### 6.1 Topic 定义 (robot_topics.hpp)

```cpp
#pragma once
#include "topic.hpp"
#include "ins_data.hpp"
#include <cstdint>

// ... enum class 定义 (ChassisMode, GimbalMode, ...) ...
// ... 命令/反馈结构体 (ChassisCmdData, GimbalCmdData, ...) ...

// IMU 姿态 (1 kHz 发布)
inline Topic<InsData>          ins_topic;

// 遥控器 (~70 Hz, UART ISR 回调发布)
inline Topic<remote::Dt7Data>  remote_topic;

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
   │  remote_topic (~70 Hz, ISR 发布)                  │
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
| `remote_topic` | UART ISR (via Remote) | cmd |
| `chassis_cmd_topic` | cmd | chassis |
| `chassis_feed_topic` | chassis | cmd |
| `gimbal_cmd_topic` | cmd | gimbal |
| `gimbal_feed_topic` | gimbal | cmd |
| `shoot_cmd_topic` | cmd | shoot |
| `shoot_feed_topic` | shoot | cmd |

## 7. 数据类型设计 (robot_topics.hpp 内)

> 以下类型定义和 Topic 实例合并在同一个 `robot_topics.hpp` 中,
> 减少文件数量, include 一个文件即可获得所有类型和 Topic。

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
- `TopicReader<remote::Dt7Data>*` (遥控器数据, ~70 Hz)
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

### 8.2 Chassis (motor_task/chassis.cpp)

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

### 8.3 Gimbal (motor_task/gimbal.cpp)

**持有资源**:
- 2 × `Motor<DjiDriver, CascadePid>` (GM6020, yaw on hcan1, pitch on hcan2)
- `TopicReader<GimbalCmdData>*`
- `TopicReader<InsData>*`
- 本地 `InsData` 缓存 (用于 IMU 反馈指针)

**IMU 反馈注入** (关键设计):
```cpp
// GimbalInit() 中:
static InsData gimbal_ins;   // 文件作用域, 供电机外部反馈指针使用

// GimbalTask() 中 (由 motor_task 1kHz 调用):
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

### 8.4 Shoot (motor_task/shoot.cpp)

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

### 8.5 Daemon (daemon_task.cpp)

**对应 basic_framework**: `daemon.c` + `buzzer.c`, 合并在 `StartDAEMONTASK` (100 Hz) 中。

**Daemon (确定性 Health Monitor)** [已实现]:

基于 FSM + 时间戳模型，每个需要监控的模块注册一个 `DaemonInstance`：

```
状态机:
  UNSEEN ──(首次 Reload)──→ ONLINE ──(超时)──→ OFFLINE
                              ↑                   │
                              └──(再次 Reload)─────┘
```

- **时间戳模型**: ISR 只写 `last_feed_tick_`（单次 32-bit 原子写），TickAll 只读比较，零竞态
- **边沿触发**: `on_offline` / `on_recover` 仅在状态迁移时触发一次
- **OFFLINE 锁存**: 超时后持续离线，直到真正恢复 Reload，`IsOnline()` 语义准确
- **自注册**: 构造即注册，固定容量静态表 (MAX_INSTANCES = 32)，模块内部零堆分配（实例由上层决定静态或堆分配）

```
使用方式:
  创建: new DaemonInstance({.timeout_ticks = 10, .on_offline = OnOffline})
  喂狗: daemon->Reload()      // ISR 安全 (单次 32-bit 写)
  查询: daemon->IsOnline()    // 消费者侧检查 (如 cmd_task 急停判断)
  驱动: DaemonInstance::TickAll()  // 100 Hz 遍历所有实例
```

**Buzzer (蜂鸣器报警)**:

优先级报警系统。多个报警源 (离线、低电压、卡弹) 按优先级注册,
`BuzzerTask()` 扫描最高优先级的活跃报警, 驱动 PWM 输出对应频率。

### 8.6 UI (ui_task.cpp)

**对应 basic_framework**: `referee_task.c`, 在 `StartUITASK` 中独立运行。

**职责**: 通过裁判系统串口绘制操作手 UI。

```
UiInit():
  初始化裁判系统串口 → 等待收到机器人 ID → 清空 UI → 绘制静态元素 (准线/标签)

UiTask():
  检测模式变化 → 按需刷新动态元素 (底盘/云台/射击状态, 功率条)
  每发一包数据 osDelay(1), 避免裁判系统通信拥塞
```

**数据来源**: 订阅 `chassis_feed_topic`, `gimbal_feed_topic`, `shoot_feed_topic`
获取当前模式和状态, 无需直接依赖任何子系统头文件。

### 8.7 视觉通信

**对应 basic_framework**: `master_process.c`, VisionSend() 在 ins_task (1 kHz) 中调用。

**设计**: 不单独建任务。
- **发送**: 在 ins_task 中, INS 更新后立即发送姿态数据 (yaw/pitch/roll), 最小化延迟
- **接收**: UART/USB 中断回调, 异步解码视觉目标数据, Daemon 监控在线状态
- **接口**: CMD 通过 Topic 或直接读取视觉数据, 融合到云台控制指令中

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

### Phase 1: 基础框架 [已完成]
1. ~~`robot_def.hpp` — 板型宏 + 硬件/机械常量~~
2. ~~`robot_topics.hpp` — 模式枚举 + 命令/反馈结构体 + Topic 实例~~
3. ~~`ins_task/` — 1 kHz IMU + EKF → Topic<InsData>~~
4. ~~`robot.cpp` — RobotInit() 入口~~
5. ~~`remote` — Remote<Protocol> 模板 + DT7 协议 + ISR 回调发布 → remote_topic~~
6. ~~`daemon` — DaemonInstance FSM + 时间戳模型 + daemon_task 100 Hz~~

### Phase 2: CMD + 底盘
5. `robot_task/cmd.cpp` — 遥控器初始化 + 输入映射
6. `robot_task/robot_task.cpp` — 200 Hz 指令任务框架（仅发布目标）
7. `motor_task/motor_task.cpp` — 1 kHz 单写者电机任务 + FlushAll

### Phase 3: 云台 + 发射
8. `motor_task/chassis.cpp` — 4 × M3508 + 麦轮运动学
9. `motor_task/gimbal.cpp` — 2 × GM6020 + IMU 反馈 (1 kHz 执行)
10. `motor_task/shoot.cpp` — 摩擦轮 + 拨弹盘

### Phase 4: 外围
11. `daemon_task/` — ~~离线检测~~ [已完成] + 蜂鸣器
12. 功率限制集成 (PowerLimiter + SuperCap)
13. 视觉通信 (ins_task 中发送, 中断接收)
14. `ui_task/` — 裁判系统 UI

### Phase 5: 多板
15. CAN 通信层封装
16. 双板条件编译测试

## 12. 关键设计模式总结

| 模式 | 说明 |
|---|---|
| **Pub-Sub 解耦** | 子系统间零 include, 全部通过 Topic<T> 通信 |
| **CMD 单点决策** | 只有 CMD 写入 mode 枚举, 子系统只读 |
| **配置合并** | robot_def (常量) + robot_topics (类型+Topic 实例), 两文件即可 |
| **TaskManager 编排** | 每个独立任务用 TaskManager lambda 封装, 生命周期清晰 |
| **文件作用域 static** | 每个 app 的状态都是 static, 等价于单例但无 class 开销 |
| **Motor 模板零虚函数** | `Motor<DjiDriver, CascadePid>` 编译期组合, 零运行时开销 |
| **ISP 防火墙** | ins_data.hpp / robot_topics.hpp 是轻量头文件, 不传递 heavy 依赖 |
| **云台 1 kHz** | 云台电机 PID 在 motor_task 中 1kHz 执行, 每周期读取最新 IMU |
| **motor_task 单写者** | 仅 motor_task 调用 `SetOutput/FlushAll`, 消除跨任务 CAN 发送竞争 |
