# 算法库 (Algorithm)

<p align='right'>powerful_framework</p>

> 控制与估计算法集合，Header-only 设计，ARM DSP 加速

---

## 目录

- [PID 控制器](#pid-控制器)
- [级联 PID](#级联-pid)
- [6-State EKF](#6-state-ekf)
- [AHRS 数学工具](#ahrs-数学工具)

---

## PID 控制器

### 如果你只需要快速使用

```cpp
#include "algorithm/pid_controller.hpp"

// 1. 配置 PID
PidConfig cfg = {
    .Kp = 10.0f,
    .Ki = 0.5f,
    .Kd = 0.0f,
    .MaxOut = 20.0f,
    .IntegralLimit = 5.0f,
    .Improve = PID_Improvement_e::Integral_Limit
};

// 2. 创建控制器
PidController pid(cfg);

// 3. 使用
float output = pid.Calculate(feedback, target, dt);  // dt 单位：秒
```

### 配置参数

```cpp
struct PidConfig {
    float Kp = 0;                  // 比例系数
    float Ki = 0;                  // 积分系数
    float Kd = 0;                  // 微分系数
    float MaxOut = 0;              // 输出限幅
    float IntegralLimit = 0;       // 积分限幅
    float DeadBand = 0;            // 死区
    float CoefA = 0;               // 变积分系数 A
    float CoefB = 0;               // 变积分系数 B
    float Output_LPF_RC = 0;       // 输出滤波 RC (1/wc)
    float Derivative_LPF_RC = 0;   // 微分滤波 RC
    PID_Improvement_e Improve = NONE;  // 改进功能位掩码
};
```

### 改进功能

```cpp
enum PID_Improvement_e {
    NONE = 0,
    Integral_Limit = 0x01,              // 积分限幅
    Derivative_On_Measurement = 0x02,   // 微分先行（D on measurement）
    Trapezoid_Integral = 0x04,          // 梯形积分
    Proportional_On_Measurement = 0x08, // 比例先行（P on measurement）
    OutputFilter = 0x10,                // 输出低通滤波
    ChangingIntegrationRate = 0x20,     // 变积分速率
    DerivativeFilter = 0x40,            // 微分低通滤波
};
```

**组合使用**:
```cpp
.Improve = Integral_Limit | Derivative_On_Measurement | Trapezoid_Integral
```

---

## 级联 PID

### 环路模式

```cpp
namespace loop_mode {
    inline constexpr uint8_t OPEN        = 0b000;  // 开环
    inline constexpr uint8_t SPEED       = 0b010;  // 仅速度环
    inline constexpr uint8_t ANGLE       = 0b100;  // 仅角度环
    inline constexpr uint8_t ANGLE_SPEED = 0b110;  // 角度→速度级联
}
```

### 使用示例

```cpp
#include "motor/controller/cascade_pid.hpp"

CascadePidConfig cfg = {
    .speed_pid = {10.0f, 0.5f, 0.0f, 20.0f},
    .angle_pid = {20.0f, 0.0f, 0.5f, 500.0f},
    .loop_mode = loop_mode::ANGLE_SPEED,
    .reverse = false
};

CascadePid controller(cfg);

// 设置目标
CascadeRef ref;
ref.target = 90.0f;        // 目标角度 90°
ref.feedforward = 100.0f;  // 速度前馈

// 计算输出
float output = controller.Compute(ref, motor_measure, dt);
```

### 运行时切换模式

```cpp
controller.SetLoopMode(loop_mode::SPEED);        // 切到速度环
controller.SetLoopMode(loop_mode::ANGLE_SPEED);  // 切回级联
```

### 外部反馈覆盖

```cpp
// 小陀螺模式：使用 IMU 角度而非电机编码器
float imu_yaw = ins_data.yaw;

FeedbackOverride fb;
fb.angle_fb = &imu_yaw;
controller.SetFeedbackOverride(fb);
```

---

## 6-State EKF

### 状态向量

```
X = [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y]ᵀ

q: 姿态四元数 (4D)
gyro_bias: 陀螺零偏 (2D，只估计 x, y 轴)
```

### 观测向量

```
Z = [ax, ay, az]ᵀ  (加速度计，归一化作为重力方向)
```

### 使用示例

```cpp
#include "algorithm/quaternion_ekf.hpp"

// 创建 EKF 实例
QuaternionEkf ekf;

// 初始化（使用初始加速度计算水平姿态）
float init_acc[3] = {0, 0, 9.8f};
ekf.Init(init_acc);

// 主循环
while (true) {
    // 读取 IMU
    float gyro[3] = {gx, gy, gz};  // rad/s
    float acc[3]  = {ax, ay, az};  // m/s²
    float dt = 0.001f;             // 1ms

    // 更新 EKF
    ekf.Update(gyro, acc, dt);

    // 获取结果
    const float* q = ekf.Quaternion();  // [q0, q1, q2, q3]
    float yaw, pitch, roll;
    ekf.GetEulerAngles(&yaw, &pitch, &roll);  // rad
}
```

### 卡方检验

EKF 内置加速度扰动检测：

```cpp
// 当加速度偏离重力过大时，自动增大观测噪声 R
// 防止运动加速度干扰姿态估计
```

### 内存占用

- 状态协方差 P: 6×6 = 36 float
- 过程噪声 Q: 6×6 = 36 float
- 观测噪声 R: 3×3 = 9 float
- 约 1440 字节/实例

---

## AHRS 数学工具

### 头文件

```cpp
#include "algorithm/ahrs_math.hpp"
```

### 常用函数

```cpp
namespace ahrs {
    // 快速倒数平方根
    float InvSqrt(float x);

    // 向量归一化
    void Norm3d(float v[3]);

    // 向量点积
    float Dot3d(const float a[3], const float b[3]);

    // 向量叉积
    void Cross3d(const float a[3], const float b[3], float out[3]);

    // 四元数 → 欧拉角
    void QuaternionToEuler(const float q[4], float* yaw, float* pitch, float* roll);

    // 欧拉角 → 四元数
    void EulerToQuaternion(float yaw, float pitch, float roll, float q[4]);

    // 重力向量 → 水平姿态四元数
    void GravityToQuaternion(const float g[3], float q[4]);
}
```

### 示例：姿态转换

```cpp
#include "algorithm/ahrs_math.hpp"

// 四元数转欧拉角
float q[4] = {0.707f, 0, 0, 0.707f};  // 90° yaw
float yaw, pitch, roll;
ahrs::QuaternionToEuler(q, &yaw, &pitch, &roll);
// yaw ≈ 1.57 rad (90°)

// 欧拉角转四元数
ahrs::EulerToQuaternion(0.0f, 0.0f, M_PI_2, q);
```

---

## 与 basic_framework 的区别

| 算法 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| PID | C 结构体 | C++ 类，支持改进功能 |
| EKF | 7-state | 6-state（更轻量） |
| 数学 | 混合实现 | namespace ahrs，Header-only |
| ARM DSP | 部分使用 | 矩阵运算全面使用 |

---

## 注意事项

1. **单位统一**: 全部使用 SI 单位（rad, rad/s, m/s²）

2. **dt 单位**: PID 和 EKF 的 dt 都是**秒**，不是毫秒

3. **初始化**: EKF 需要正确初始化，建议用 100 个样本平均加速度

4. **Z 轴零偏**: EKF 只估计 X/Y 轴零偏，Z 轴偏航由加速度约束

5. **ARM DSP**: 需要链接 CMSIS-DSP 库，CMake 已配置
