# motor_task

电机控制执行任务, 1 kHz FreeRTOS 任务。

## 总览

`motor_task` 是所有电机的统一控制入口, 按板型条件编译包含不同子系统:

| 板型 | 子系统 |
|---|---|
| `ONE_BOARD` | ChassisMotors + GimbalMotors + ShootMotors |
| `GIMBAL_BOARD` | GimbalMotors + ShootMotors |
| `CHASSIS_BOARD` | ChassisMotors |

## 工作流程

### 初始化

1. **Pre-scheduler 阶段** (在 `MotorTaskStart()` 中, `TaskManager` 构造前): 调用各子系统 `Init()`, 创建电机实例并订阅 Topic
2. **Task init_func**: 预热 DWT 计时器

### 循环 (1 kHz)

1. `DwtGetDeltaT()` 获取精确 dt
2. 调用各子系统 `Tick(dt)` (全部 1 kHz)
3. 统一 `FlushAll()` 发送所有驱动类型的控制帧

## 代码结构

| 文件 | 说明 |
|---|---|
| `motor_task.hpp` | `MotorTaskStart()` 声明 |
| `motor_task.cpp` | TaskManager + 板级条件编译调度 |
| `chassis_motors.hpp/cpp` | 底盘 4×M3508 + 麦轮分解 + 功率限制 |
| `gimbal_motors.hpp/cpp` | 云台 2×GM6020 + 串级 PID + INS 反馈覆盖 |
| `shoot_motors.hpp/cpp` | 发射 2×M3508 摩擦轮 + 1×M2006 拨弹盘 |

## 子系统

### ChassisMotors

- 4×M3508 (CAN2), 速度环 PID
- 通过 `TopicReader<ChassisCmdData>` 接收底盘命令
- 麦克纳姆轮逆运动学分解
- `PowerLimiter` 三层功率控制 (RLS + 能量环 + 二次分配)

### GimbalMotors

- 2×GM6020 Yaw + Pitch (CAN1), 角度→速度串级 PID
- 通过 `TopicReader<GimbalCmdData>` 接收云台命令
- 通过 `TopicReader<InsData>` 获取 IMU 姿态, 覆盖编码器角度反馈
- 发布 `GimbalFeedData` 到 `gimbal_feed_topic`

### ShootMotors

- 2×M3508 摩擦轮 + 1×M2006 拨弹盘 (CAN2)
- 通过 `TopicReader<ShootCmdData>` 接收发射命令
- 支持模式: STOP / REVERSE / SINGLE / TRIPLE / BURST
- 根据模式自动切换 `loop_mode` (SPEED / ANGLE_SPEED)

## 数据流

```
chassis_cmd_topic ──→ ChassisMotors::Tick() ──→ DjiDriver CAN 缓冲
gimbal_cmd_topic  ──→ GimbalMotors::Tick() ──→ DjiDriver CAN 缓冲 ──→ gimbal_feed_topic
shoot_cmd_topic   ──→ ShootMotors::Tick()  ──→ DjiDriver CAN 缓冲
ins_topic ─────────→ GimbalMotors (反馈覆盖)
                                                DjiDriver::FlushAll() ← 统一发送
```
