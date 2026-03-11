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
├── general_def.hpp        # 数学常量 (PI, 角度转换) + LPF + Clamp
├── topic.hpp              # Topic<T> 单写者多读者 SeqLock 发布订阅
├── algorithm/             # PID 控制器, 四元数 EKF, AHRS 数学工具
├── imu/                   # BMI088 驱动 + INS 姿态解算
├── motor/                 # Motor<Driver, Controller> 电机系统
├── remote/                # Remote<Protocol> 遥控器模板
└── daemon/                # DaemonInstance 健康监测 (FSM + 时间戳模型)
```

## 通用组件

| 文件 | 说明 |
|---|---|
| `general_def.hpp` | `PI`, `RAD_2_DEGREE`, `ECD_ANGLE_COEF`, `Clamp()`, `LowPassFilter()` |
| `topic.hpp` | `Topic<T, MaxSubs>` SeqLock 无锁发布订阅 + `TopicReader<T>` 独立追踪序号 |
