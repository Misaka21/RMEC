# 电机系统 (Motor System)

<p align='right'>powerful_framework</p>

> 零虚函数设计的模板组合电机系统，支持 DJI、DM、HT、LK 等多种电机驱动。

---

## 如果你只需要快速使用

电机系统采用**模板组合**设计：`Motor<Driver, Controller>`

- **Driver** 负责 CAN 协议编解码（硬件相关）
- **Controller** 负责控制算法（PID、前馈等）

### 最简单的使用示例

```cpp
#include "motor/motor.hpp"
#include "motor/driver/dji_driver.hpp"
#include "motor/controller/cascade_pid.hpp"

// 1. 配置 DJI 电机驱动
DjiDriverConfig dji_cfg = {
    .motor_type = DjiMotorType::M3508,
    .can_handle = &hcan2,
    .motor_id = 1  // 电调闪动次数
};

// 2. 配置级联 PID 控制器
CascadePidConfig pid_cfg = {
    .speed_pid = {.Kp = 10.0f, .Ki = 0.5f, .Kd = 0.0f, .MaxOut = 20.0f},
    .angle_pid = {.Kp = 20.0f, .Ki = 0.0f, .Kd = 0.5f, .MaxOut = 500.0f},
    .loop_mode = loop_mode::ANGLE_SPEED,  // 角度→速度级联
    .reverse = false
};

// 3. 创建电机实例
Motor<DjiDriver, CascadePid> motor(dji_cfg, pid_cfg);

// 4. 在任务循环中控制
void MotorTask() {
    motor.Enable();
    while (true) {
        motor.SetRef(CascadeRef{90.0f});  // 设置目标角度 90°
        motor.Update(0.001f);              // 更新控制 (1ms 周期)
        DjiDriver::FlushAll();             // 批量发送 CAN 报文
        vTaskDelay(1);
    }
}
```

---

## 系统架构

```
Motor<Driver, Controller>
       │           │
       ▼           ▼
   ┌───────┐   ┌──────────┐
   │Driver │   │Controller│
   │(CAN协议)│   │  (PID等)  │
   └───┬───┘   └────┬─────┘
       │            │
   ┌───▼───┐   ┌───▼────┐
   │CANInstance│ │MotorMeasure│
   └────────┘   └─────────┘
```

### 支持的 Driver

| Driver | 电机类型 | 控制量 |
|--------|---------|--------|
| `DjiDriver` | M3508, M2006, GM6020 | 电流(A) / 电压(V) |
| `DmDriver` | DM 系列 | MIT 协议 |
| `HtDriver` | HT04 等 | MIT 协议 |
| `LkDriver` | LK 系列 | 自定义协议 |

### 支持的 Controller

| Controller | 功能 |
|------------|------|
| `CascadePid` | 级联角度→速度 PID |
| `MitPassthrough` | MIT 协议透传 |
| `MpcTracker` | MPC 轨迹跟踪 |

---

## MotorMeasure - 电机反馈数据

```cpp
struct MotorMeasure {
    float angle_deg;      // 单圈角度 [0, 360)
    float total_angle;    // 多圈累计角度（可正负）
    float speed_rpm;      // 转速 RPM
    float speed_aps;      // 转速 度/秒
    float current;        // 实际电流 (A)
    uint8_t temperature;  // 温度 (°C)
};
```

### 获取电机反馈

```cpp
const MotorMeasure& m = motor.Measure();
float current_speed = m.speed_aps;
float total_angle = m.total_angle;
```

---

## CascadePid - 级联 PID 控制器

### 环路模式 (loop_mode)

```cpp
namespace loop_mode {
    inline constexpr uint8_t OPEN        = 0b000;  // 开环
    inline constexpr uint8_t SPEED       = 0b010;  // 仅速度环
    inline constexpr uint8_t ANGLE       = 0b100;  // 仅角度环
    inline constexpr uint8_t ANGLE_SPEED = 0b110;  // 角度→速度级联
}
```

### 运行时切换模式

```cpp
// 底盘电机：正常时速度环，小陀螺时角度环
motor.GetController().SetLoopMode(loop_mode::SPEED);   // 速度模式
motor.GetController().SetLoopMode(loop_mode::ANGLE_SPEED); // 级联模式
```

### 外部反馈覆盖

用于小陀螺模式（云台使用 IMU 角度而非电机编码器）：

```cpp
float imu_yaw = 0.0f;  // 从 INS 获取

// 设置外部反馈指针
FeedbackOverride fb;
fb.angle_fb = &imu_yaw;
motor.GetController().SetFeedbackOverride(fb);
```

---

## PID 控制器参数

```cpp
struct PidConfig {
    float Kp = 0;           // 比例系数
    float Ki = 0;           // 积分系数
    float Kd = 0;           // 微分系数
    float MaxOut = 0;       // 输出限幅
    float IntegralLimit = 0;    // 积分限幅
    float DeadBand = 0;         // 死区
    float CoefA = 0;            // 变积分系数 A
    float CoefB = 0;            // 变积分系数 B
    float Output_LPF_RC = 0;    // 输出滤波 RC
    float Derivative_LPF_RC = 0; // 微分滤波 RC
    PID_Improvement_e Improve = NONE;  // 改进功能位掩码
};
```

### 改进功能选项

```cpp
enum PID_Improvement_e {
    NONE = 0,
    Integral_Limit = 0x01,           // 积分限幅
    Derivative_On_Measurement = 0x02, // 微分先行
    Trapezoid_Integral = 0x04,       // 梯形积分
    Proportional_On_Measurement = 0x08, // 比例先行
    OutputFilter = 0x10,             // 输出滤波
    ChangingIntegrationRate = 0x20,  // 变积分
    DerivativeFilter = 0x40,         // 微分滤波
};
```

---

## 两阶段 API（功率控制场景）

当需要根据电容电量动态限制功率时：

```cpp
// 1. 计算输出（不发送）
float output = motor.ComputeOutput(0.001f);

// 2. 功率缩放
float power_scale = GetPowerScale();  // 根据电容电压计算
output *= power_scale;

// 3. 应用输出
motor.ApplyOutput(output);

// 4. 批量发送
DjiDriver::FlushAll();
```

---

## DJI 电机 Driver 详解

### 电机 ID 与 CAN ID 映射

| 电机 ID | M3508/M2006 TX ID | GM6020 TX ID |
|---------|-------------------|--------------|
| 1 | 0x200 | 0x1FF |
| 2 | 0x200 | 0x1FF |
| 3 | 0x200 | 0x1FF |
| 4 | 0x200 | 0x1FF |
| 5 | 0x1FF | 0x2FF |
| 6 | 0x1FF | 0x2FF |
| 7 | 0x1FF | 0x2FF |
| 8 | 0x1FF | 0x2FF |

> 无需手动计算！初始化时只需填写 `motor_id = 1~8`，Driver 自动处理。

### 物理单位换算

| 电机 | SetOutput 单位 | 内部换算 |
|------|---------------|---------|
| M3508 | 电流 (A) | raw = A × 819.2 |
| M2006 | 电流 (A) | raw = A × 1000 |
| GM6020 | 电压 (V) | raw = V × 1250 |
| GM6020_CURRENT | 电流 (A) | raw = A × 5461.3 |

---

## 完整示例：底盘电机配置

```cpp
// 配置 M3508 底盘电机（4个）
struct MotorConfig {
    DjiMotorType type;
    uint8_t id;
    PidConfig speed_pid;
    bool reverse;
};

MotorConfig chassis_motor_cfgs[4] = {
    {DjiMotorType::M3508, 1, {15.0f, 0.3f, 0.0f, 20.0f}, false},  // 左前
    {DjiMotorType::M3508, 2, {15.0f, 0.3f, 0.0f, 20.0f}, true},   // 右前（反转）
    {DjiMotorType::M3508, 3, {15.0f, 0.3f, 0.0f, 20.0f}, true},   // 左后（反转）
    {DjiMotorType::M3508, 4, {15.0f, 0.3f, 0.0f, 20.0f}, false},  // 右后
};

// 创建电机数组
using ChassisMotor = Motor<DjiDriver, CascadePid>;
ChassisMotor* chassis_motors[4];

void ChassisMotorsInit() {
    for (int i = 0; i < 4; i++) {
        DjiDriverConfig d_cfg = {
            .motor_type = chassis_motor_cfgs[i].type,
            .can_handle = &hcan2,
            .motor_id = chassis_motor_cfgs[i].id
        };
        CascadePidConfig c_cfg = {
            .speed_pid = chassis_motor_cfgs[i].speed_pid,
            .loop_mode = loop_mode::SPEED
        };
        chassis_motors[i] = new ChassisMotor(d_cfg, c_cfg);
        chassis_motors[i]->Enable();
    }
}

void ChassisTask() {
    while (true) {
        // 设置速度（由逆解算得到）
        for (int i = 0; i < 4; i++) {
            chassis_motors[i]->SetRef(CascadeRef{target_speeds[i]});
            chassis_motors[i]->Update(0.005f);  // 200Hz
        }
        DjiDriver::FlushAll();
        vTaskDelay(5);  // 5ms = 200Hz
    }
}
```

---

## 离线检测

每个 Driver 内置离线检测：

```cpp
// 查询电机是否在线
if (motor.IsOnline()) {
    motor.Update(dt);
} else {
    // 离线处理：电机停止、报警等
    motor.Disable();
}
```

离线阈值：`DjiDriver::OFFLINE_THRESHOLD = 100`（100次 TickOffline 未收到反馈视为离线）

---

## 注意事项

1. **必须调用 FlushAll()**: DJI 电机使用分组发送缓冲，每个控制周期结束时必须调用 `DjiDriver::FlushAll()`

2. **Enable/Disable**: 电机默认禁用，使用前需调用 `Enable()`

3. **控制频率**: 建议 500Hz~1kHz，与电机反馈频率匹配

4. **PID 参数单位**:
   - 速度环：输入 deg/s，输出 A
   - 角度环：输入 deg，输出 deg/s

5. **多圈角度**: 电机上电时编码器位置为 0°，正转增加，反转减少，可累计超过 360°
