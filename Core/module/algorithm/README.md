# algorithm

算法模块, 提供控制和姿态估计所需的基础算法。

## 代码结构

| 文件 | 说明 |
|---|---|
| `pid_controller.hpp/cpp` | 位置式 PID 控制器, 支持 8 种改进项 |
| `quaternion_ekf.hpp/cpp` | 6 状态四元数扩展卡尔曼滤波, 基于 ARM DSP |
| `ahrs_math.hpp` | header-only AHRS 数学工具 (向量运算, 坐标变换) |

## PID 控制器

通过 `improve_flags` 位域组合启用改进项:

| 标志 | 功能 |
|---|---|
| `INTEGRAL_LIMIT` | 积分限幅 |
| `DERIVATIVE_ON_MEASUREMENT` | 微分先行 (D-on-measurement) |
| `TRAPEZOIDAL_INTEGRAL` | 梯形积分 |
| `PROPORTIONAL_ON_MEASUREMENT` | 比例先行 (P-on-measurement) |
| `OUTPUT_FILTER` | 输出一阶 LPF |
| `CHANGING_INTEGRATION_RATE` | 变速积分 |
| `DERIVATIVE_FILTER` | 微分一阶 LPF |
| `ERROR_HANDLE` | 堵转检测 |

### 外部接口

```cpp
PidController(const PidConfig& cfg);
float Calculate(float measure, float ref, float dt);
void  Reset();
float Output() const;
```

## 四元数 EKF

6 状态: 四元数 q[4] + 陀螺仪零飘 gyro_bias[2], 约 1440 字节/实例。

- 渐消因子 `lambda` 抑制发散
- 卡方检验拒绝加速度异常量测
- 加速度计 LPF 预滤波 (可选)

### 外部接口

```cpp
void Init(const float init_q[4], const QuaternionEkfConfig& cfg);
void Update(float gx, float gy, float gz,
            float ax, float ay, float az, float dt);
const QuaternionEkfOutput& Output() const;
```

## AHRS 数学工具

header-only, `namespace ahrs`:

```cpp
float InvSqrt(float x);                    // 快速逆平方根
void  Norm3d(float v[3]);                   // 归一化
void  BodyFrameToEarthFrame(...)            // 体→地坐标变换
void  EarthFrameToBodyFrame(...)            // 地→体坐标变换
void  GravityToQuaternion(acc_avg, q_out);  // 加速度→初始四元数
```
