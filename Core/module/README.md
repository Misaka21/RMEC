# module

模块层, 位于 SAL 与 App 之间, 提供与具体硬件解耦的功能组件。

## 设计原则

- **零虚函数**: 全部使用模板组合实现多态, 编译期确定策略
- **不持有 Topic**: 模块是纯数据源/算法, 数据流向由 App 层编排
- **SAL 实例由 `new` 创建**: 生命周期全局, 禁止手动 `delete`
- **配置用纯数据结构体**: aggregate init, 公开字段不带尾随 `_`

## 目录结构

```
module/
├── math.hpp               # 数学常量 (PI, 角度转换) + LPF + Clamp (namespace math)
├── topic.hpp              # Topic<T> 单写者多读者 SeqLock 发布订阅
├── algorithm/             # PID 控制器, 四元数 EKF, AHRS 数学工具
├── imu/                   # BMI088 驱动 + INS 姿态解算
├── motor/                 # Motor<Driver, Controller> 电机系统
├── remote/                # Remote<Protocol> 遥控器模板
├── daemon/                # DaemonInstance 健康监测 (FSM + 时间戳模型)
└── can_comm/              # CanComm<TxData, RxData> 多 ID 映射双板通信
```

## 模块文档

| 模块 | 文档 | 说明 |
|------|------|------|
| motor | [motor/motor.md](motor/motor.md) | 电机系统，支持 DJI/DM/HT/LK |
| imu | [imu/imu.md](imu/imu.md) | BMI088 驱动 + INS 姿态解算 |
| remote | [remote/remote.md](remote/remote.md) | DT7 遥控器，模板策略模式 |
| daemon | [daemon/daemon.md](daemon/daemon.md) | 健康监测守护进程 |
| can_comm | [can_comm/can_comm.md](can_comm/can_comm.md) | 双板 CAN 通信 |
| vision | [vision/vision.md](vision/vision.md) | 视觉通信协议 |
| referee | [referee/referee.md](referee/referee.md) | 裁判系统协议解析 |
| algorithm | [algorithm/algorithm.md](algorithm/algorithm.md) | PID、EKF、AHRS 数学工具 |
| topic | [topic.md](topic.md) | 发布订阅机制 |

---

## 快速上手

新手建议阅读顺序：
1. [topic.md](topic.md) - 理解数据如何流转
2. [daemon/daemon.md](daemon/daemon.md) - 理解离线检测机制
3. [motor/motor.md](motor/motor.md) - 控制电机
4. [imu/imu.md](imu/imu.md) - 姿态解算

---

## 通用组件

| 文件 | 说明 |
|---|---|
| `math.hpp` | `math::PI`, `math::RAD_2_DEGREE`, `math::ECD_ANGLE_COEF`, `math::Clamp()`, `math::LowPassFilter()` |
| `topic.hpp` | `Topic<T, MaxSubs>` SeqLock 无锁发布订阅 + `TopicReader<T>` 独立追踪序号 |
