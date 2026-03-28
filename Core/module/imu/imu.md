# BMI088 + INS 姿态解算

<p align='right'>powerful_framework</p>

> 高精度 6 轴 IMU 驱动 + 扩展卡尔曼滤波姿态估计

---

## 如果你只需要快速使用

```cpp
#include "imu/bmi088.hpp"
#include "imu/ins.hpp"
#include "app/robot_topics.hpp"
#include "sal_dwt.h"

void InsTask() {
    // 1. 配置 BMI088
    Bmi088Config cfg = {
        .work_mode = Bmi088WorkMode::BLOCK_PERIODIC,
        .cali_mode = Bmi088CaliMode::ONLINE,
        .spi_handle = &hspi1,
        .acc_cs_port = GPIOA, .acc_cs_pin = GPIO_PIN_4,
        .gyro_cs_port = GPIOB, .gyro_cs_pin = GPIO_PIN_0,
        .heat_pid_config = {.Kp = 5.0f, .Ki = 0.1f, .MaxOut = 1000.0f},
        .heat_tim_handle = &htim3,
        .heat_tim_channel = TIM_CHANNEL_1,
        .heat_target_temp = 45.0f
    };

    // 2. 创建 IMU 实例（阻塞约 6 秒完成初始化+校准）
    Bmi088 imu(cfg);

    // 3. 初始化 INS（读取 100 样本计算初始姿态）
    Ins ins;
    Bmi088Data data;
    for (int i = 0; i < 100; i++) {
        while (!imu.Acquire(data)) vTaskDelay(1);
        ins.Init(data);
    }

    // 4. DWT 计时
    DwtInstance dwt;
    float last_time = dwt.GetDeltaTime();

    // 5. 主循环
    while (true) {
        float now = dwt.GetDeltaTime();
        float dt = now - last_time;
        last_time = now;

        // 读取 IMU 数据
        if (imu.Acquire(data)) {
            // INS 解算
            InsData result = ins.Update(data, dt);

            // 发布到 Topic
            ins_topic.Publish(result);
        }

        // 温控（每 100ms）
        static int cnt = 0;
        if (++cnt >= 100) {
            imu.HeaterCtrl(0.1f);
            cnt = 0;
        }

        vTaskDelay(1);  // 1kHz
    }
}
```

---

## BMI088 驱动

### 两种工作模式

| 模式 | 说明 | 适用场景 |
|------|------|----------|
| `BLOCK_PERIODIC` | 阻塞轮询读取 | 简单应用，无 EXTI 引脚 |
| `BLOCK_TRIGGER` | EXTI 触发 + ISR 内读取 | 高精度同步，推荐 |

### 关键数据结构

```cpp
struct Bmi088Data {
    float gyro[3];      // rad/s, 已减零偏
    float acc[3];       // m/s², 已校准
    float temperature;  // °C
};
```

坐标系：
- X: 向右
- Y: 向前
- Z: 向上

### 校准模式

```cpp
enum class Bmi088CaliMode {
    ONLINE,         // 上电在线校准（约 6 秒，需静止）
    PRE_CALIBRATED  // 使用预存校准参数
};
```

**在线校准流程**:
1. 采集 6000 样本计算陀螺零偏
2. 运动检测自动重试
3. 12 秒超时回退到预存参数

### 温控系统

```cpp
// 在慢速循环中调用（建议 10Hz）
void HeaterCtrl(float dt);  // dt 为调用间隔（秒）
```

加热器使用 PID 控制，目标温度默认 40°C，建议范围 40~50°C。

---

## INS 姿态解算

### 纯函数设计

INS 类是**无状态转换器**：输入 (gyro, acc, dt) → 输出姿态数据

```cpp
class Ins {
public:
    // 使用初始加速度计算初始四元数（水平放置时）
    void Init(const Bmi088Data& data);

    // 更新姿态，返回 InsData
    InsData Update(const Bmi088Data& data, float dt);
};
```

### 输出数据

```cpp
struct InsData {
    // 欧拉角 (rad)
    float yaw;    // 偏航角
    float pitch;  // 俯仰角
    float roll;   // 横滚角

    // 四元数
    float q[4];

    // 角速度 (rad/s)
    float gyro[3];

    // 加速度 (m/s²)
    float acc[3];
};
```

### 数据流

```
BMI088 → Acquire() → Bmi088Data
                          ↓
                    EKF Update (6-state)
                          ↓
                    quaternion_ekf.hpp
                          ↓
                    Euler angles conversion
                          ↓
                    InsData → Topic Publish
```

---

## 6-State EKF 算法

### 状态向量

```
X = [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y]ᵀ
```

- 4 维：姿态四元数
- 2 维：陀螺仪零偏（x, y 轴）

### 观测向量

```
Z = [ax, ay, az]ᵀ  (加速度计，归一化作为重力方向)
```

### 关键特性

| 特性 | 说明 |
|------|------|
| 卡方检验 | 检测加速度扰动，动态调整观测噪声 |
| 零偏估计 | 在线估计并补偿陀螺零偏漂移 |
| ARM DSP | 使用 `arm_mat_mult_f32` 等加速矩阵运算 |
| 内存占用 | 约 1440 字节/实例 |

---

## 与 basic_framework 的区别

| 特性 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| BMI088 | C 结构体 + 函数 | C++ 类，RAII |
| 校准 | 固定样本数 | 运动检测 + 超时回退 |
| INS | 全局状态 | 纯函数，无状态 |
| 姿态输出 | 角度制 | 弧度制（SI） |
| EKF | 7-state | 6-state（更轻量） |
| 数据发布 | 全局变量 | Topic 发布订阅 |

---

## 完整示例：1kHz 姿态任务

```cpp
// ins_task.cpp
#include "imu/bmi088.hpp"
#include "imu/ins.hpp"
#include "app/robot_topics.hpp"
#include "sal_dwt.h"

static Bmi088* imu = nullptr;  // 静态指针供其他地方访问

void InsTaskStart() {
    // 配置（根据实际硬件修改引脚）
    Bmi088Config cfg = {
        .work_mode = Bmi088WorkMode::BLOCK_PERIODIC,
        .cali_mode = Bmi088CaliMode::ONLINE,
        .spi_handle = &hspi1,
        .acc_cs_port = ACC_CS_GPIO_Port,
        .acc_cs_pin = ACC_CS_Pin,
        .gyro_cs_port = GYRO_CS_GPIO_Port,
        .gyro_cs_pin = GYRO_CS_Pin,
        .heat_pid_config = {.Kp = 5.0f, .Ki = 0.1f, .MaxOut = 1000.0f},
        .heat_tim_handle = &htim3,
        .heat_tim_channel = TIM_CHANNEL_1,
        .heat_target_temp = 45.0f
    };

    // 创建 IMU（阻塞约 6 秒）
    imu = new Bmi088(cfg);

    xTaskCreate([](void*) {
        Ins ins;
        Bmi088Data data;

        // 初始化 EKF
        for (int i = 0; i < 100; i++) {
            while (!imu->Acquire(data)) vTaskDelay(1);
            ins.Init(data);
        }

        DwtInstance dwt;
        float last_time = dwt.GetDeltaTime();
        int heat_cnt = 0;

        while (true) {
            float now = dwt.GetDeltaTime();
            float dt = now - last_time;
            last_time = now;

            if (imu->Acquire(data)) {
                InsData result = ins.Update(data, dt);
                ins_topic.Publish(result);
            }

            if (++heat_cnt >= 100) {
                imu->HeaterCtrl(0.1f);
                heat_cnt = 0;
            }

            vTaskDelay(1);
        }
    }, "InsTask", 512, nullptr, 4, nullptr);
}

Bmi088* GetImu() { return imu; }
```

---

## 注意事项

1. **校准需静止**: 在线校准时机器人必须静止，否则会检测运动并重试

2. **初始化时间**: 构造函数阻塞约 6 秒（初始化 + 校准），建议在任务中创建

3. ** heater 频率**: `HeaterCtrl()` 不需要高频调用，10Hz 足够

4. **Z 轴零偏**: EKF 只估计 X/Y 轴陀螺零偏，Z 轴偏航由加速度辅助约束

5. **加速度扰动**: 剧烈运动时（撞击、跳跃）加速度计受干扰，EKF 卡方检验会自动降低置信度

6. **单位统一**: 框架内全部使用**弧度制**（rad, rad/s），与 HAL 的 degree 不同
