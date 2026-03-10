# imu

BMI088 六轴 IMU 驱动 + INS 姿态解算。

## 代码结构

| 文件 | 说明 |
|---|---|
| `bmi088_reg.hpp` | 寄存器地址, 灵敏度常量, 错误码 (`namespace bmi088`) |
| `bmi088.hpp/cpp` | `Bmi088` 类: SPI 驱动, 校准, EXTI 触发, 加热器 PID |
| `ins_data.hpp` | `InsData` + `InsConfig` 轻量数据结构 (供 Topic 传输) |
| `ins.hpp` | `Ins` 类: header-only, (gyro, accel, dt) → InsData 纯变换器 |

## 层级关系

```
Bmi088 (硬件驱动, 产出 Bmi088Data)
  ↓
Ins (纯算法, 消费 gyro+accel, 产出 InsData)
  ↓
App 层 ins_task 编排两者, 发布到 ins_topic
```

`Ins` 不持有 `Bmi088&` / `Topic&` / `DwtInstance` — 全部由 App 层注入, 遵循依赖倒置。

## Bmi088

### 工作模式

| 模式 | 说明 |
|---|---|
| `BLOCK_PERIODIC` | 任务轮询调用 `Acquire()`, 阻塞等待 SPI 完成 |
| `BLOCK_TRIGGER` | EXTI 中断触发读取 (更精确的时间戳) |

### 校准模式

| 模式 | 说明 |
|---|---|
| `ONLINE` | 上电静止 6000 样本在线校准 (约 6s), 运动检测, 超时回退预校准 |
| `PRE_CALIBRATED` | 跳过在线校准, 直接使用 `Bmi088PreCali` 中的预设值 |

### SPI 协议

- 加速度计: 发送 `len+2` 字节, 有效数据从 `rx[2]` 开始 (dummy byte)
- 陀螺仪: 发送 `len+1` 字节, 有效数据从 `rx[1]` 开始

### 外部接口

```cpp
Bmi088(const Bmi088Config& cfg);    // 构造即初始化+校准
bool Acquire(Bmi088Data& out);      // 读取传感器数据
void HeaterCtrl(float dt);          // 温度 PID 控制
```

## Ins

header-only 姿态解算器, 包装 `QuaternionEkf`:

```cpp
void Init(const float init_q[4], const InsConfig& cfg = {});
void Update(const float gyro[3], const float accel[3], float dt, float temperature);
const InsData& Data() const;
```

输出 `InsData`: 欧拉角、四元数、连续 yaw、体/地坐标系运动加速度。
