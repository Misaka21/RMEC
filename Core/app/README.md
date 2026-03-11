# application

应用层, 负责任务编排、数据流定义和机器人行为逻辑。

## 设计原则

- **单写者 Topic**: 每个 Topic 只有一个任务/ISR 写入, 使用 SeqLock 无锁通信
- **App 之间不互相 include**: 通过 `robot_topics.hpp` 中的 Topic 实例解耦
- **Module 不知道 App**: Module 是纯数据源/算法, App 编排数据流向

## 目录结构

```
app/
├── robot.cpp             # RobotInit() — C++ 侧唯一入口
├── robot_def.hpp         # 硬件映射和机器人参数 (板型, 引脚, PID 参数)
├── robot_topics.hpp      # 全局 Topic 实例 (数据流清单)
├── ins_task/             # IMU 姿态估计任务 (1 kHz FreeRTOS 任务)
├── remote_task/          # 遥控器初始化 + ISR 发布 + Daemon 离线检测
└── daemon_task/          # 看门狗任务 (100 Hz, DaemonInstance::TickAll)
```

## 数据流

```
                    ┌─────────────────────────────────────┐
                    │           robot_topics.hpp          │
                    │                                     │
  ins_task ──────── │ → ins_topic (1 kHz)                 │
  UART ISR ──────── │ → remote_topic (~70 Hz, ISR 回调)    │
                    │                                     │
  (未来) cmd_task ── │ → chassis_cmd / gimbal_cmd / shoot  │ ──→ 子系统 task
  子系统 task ────── │ → chassis_feed / gimbal_feed        │ ──→ cmd_task
                    └─────────────────────────────────────┘
```

## 启动流程

```
main.c
  → MX_xxx_Init()          // CubeMX 外设初始化
  → RobotInit()            // C++ 入口 (robot.cpp)
      → InsTaskStart()     // 创建 1 kHz IMU 任务
      → RemoteInit()       // 创建遥控器 + Daemon, 启动 UART DMA
      → DaemonTaskStart()  // 启动 100Hz 看门狗任务
  → osKernelStart()        // 启动调度器
```

## robot_def.hpp

集中定义机器人硬件参数:

- 板型选择: `ONE_BOARD` / `CHASSIS_BOARD` / `GIMBAL_BOARD`
- IMU 引脚映射、PID 参数、预校准值
- 遥控器 UART 句柄
- 云台/底盘/发射机械参数

## robot_topics.hpp

全局 Topic 实例清单, 定义了整车的数据流:

| Topic | 频率 | 发布者 | 数据类型 |
|---|---|---|---|
| `ins_topic` | 1 kHz | ins_task | `InsData` |
| `remote_topic` | ~70 Hz | UART ISR (via Remote) | `Dt7Data` |
| `chassis_cmd_topic` | 200 Hz | cmd_task | `ChassisCmdData` |
| `gimbal_cmd_topic` | 200 Hz | cmd_task | `GimbalCmdData` |
| `shoot_cmd_topic` | 200 Hz | cmd_task | `ShootCmdData` |
| `chassis_feed_topic` | 200 Hz | chassis_task | `ChassisFeedData` |
| `gimbal_feed_topic` | 200 Hz | gimbal_task | `GimbalFeedData` |
| `shoot_feed_topic` | 200 Hz | shoot_task | `ShootFeedData` |
