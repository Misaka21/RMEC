# motor

电机模块, 使用模板组合 `Motor<Driver, Controller>` 实现零虚函数多态。

## 总览

用户只需选择 Driver + Controller 组合:

```cpp
using DjiCascadeMotor = Motor<DjiDriver, CascadePid>;    // DJI 电机 + 串级 PID
using DjiMitMotor     = Motor<DjiDriver, MitPassthrough>; // DJI 电机 + 直通 (外部 PID)
```

Motor 封装了:
- 驱动通信 (CAN 收发, 编码器解析)
- 控制算法 (PID 计算 / 直通)
- 离线检测
- 两阶段输出 API (支持功率限制)

## 代码结构

```
motor/
├── motor.hpp                    # Motor<D,C> 模板主类
├── motor_measure.hpp            # MotorMeasure 编码器数据 + 多圈角度
├── driver/
│   └── dji_driver.hpp/cpp       # DJI M3508/M2006/GM6020/GM6020_CURRENT CAN 驱动
├── controller/
│   ├── cascade_pid.hpp/cpp      # 角度→速度串级 PID
│   └── mit_passthrough.hpp      # 直通控制器 (MIT 模式)
└── power_limiter.hpp/cpp        # 三层功率控制 (RLS + 能量环 + 二次规划)
```

## Driver 接口契约

Driver 需提供:

```cpp
void                SetOutput(float output);
const MotorMeasure& Measure() const;
MotorMeasure&       MeasureMut();
void                TickOffline();
bool                IsOnline() const;
```

### DjiDriver

- 支持 M3508, M2006, GM6020 (电压), GM6020_CURRENT (电流)
- 10 个静态 TxGroup, 按 CAN 总线 + 仲裁 ID + 控制模式分组
- CAN ISR 回调自动更新 `MotorMeasure`
- `FlushAll()` 一次性发送所有组的控制帧, 每个控制周期调用一次

## Controller 接口契约

Controller 需提供:

```cpp
float Compute(float ref, const MotorMeasure& measure, float dt);
```

### CascadePid

串级 PID, 支持四种环路模式:

| 模式 | 说明 |
|---|---|
| `OPEN` | 开环直通 |
| `SPEED` | 单速度环 |
| `ANGLE` | 单角度环 |
| `ANGLE_SPEED` | 角度→速度串级 (默认) |

支持外部反馈覆盖 (`FeedbackOverride`), 如用 IMU yaw 替代编码器角度。

### MitPassthrough

直通控制器, `Compute()` 直接返回 ref, 用于电机端已有 PID 的场景 (HT/DM 电机)。

## 两阶段 API

用于功率限制场景:

```cpp
float output = motor.ComputeOutput(dt);   // 1. 计算控制输出
// ... 功率限制修改 output ...
motor.ApplyOutput(output);                 // 2. 发送到驱动
```

## PowerLimiter

三层功率控制:

1. **能量环**: PD 控制缓冲能量, 计算动态最大功率
2. **RLS 在线辨识**: 实时估计电机功率模型参数 k1, k2
3. **二次功率分配**: 按误差/比例混合权重 + 二次求解, 修改各电机输出

```cpp
limiter.UpdateEnergyLoop(power_limit, buffer_energy, measured_power, dt);
limiter.Limit(motor_states, count);   // 就地修改 pid_output
```
