# SAL — Software Abstraction Layer

SAL 层是对 STM32 片上外设的 C++ 封装，为上层 module 层提供统一、安全的硬件访问接口。

> 与 HAL 的关系：CubeMX 生成的 HAL 初始化代码位于 `Inc/` 和 `Src/`，SAL 在其之上进一步封装，**module 层不应直接调用 HAL 函数**。

---

## 设计原则

1. **实例化注册制**：通信类外设（CAN、UART、SPI、I2C）通过构造 `XxxInstance` 完成注册，每个外设连接的设备对应一个实例。构造时传入 `XxxConfig` 配置结构体。
2. **回调驱动**：接收完成、发送完成等事件通过 `std::function` 回调通知 module 层，回调在中断上下文中执行（注意不要做耗时操作）。
3. **单例/静态工具**：只有唯一硬件资源的模块（DWT、Flash、USB、Log）使用静态方法或单例模式，不需要手动管理实例列表。
4. **命名空间**：除 `DWTInstance` 外，所有 SAL 类型位于 `namespace sal` 中。

---

## 模块总览

| 模块 | 头文件 | 类型 | 说明 |
|---|---|---|---|
| CAN | `can/sal_can.h` | 多实例 | CAN1/CAN2 总线通信，FIFO 发送队列 |
| UART | `usart/sal_usart.h` | 多实例 | UART1/3/6，DMA+IDLE 接收，FIFO 发送队列 |
| SPI | `spi/sal_spi.h` | 多实例 | SPI1/2，软件 CS 管理，寄存器读写便捷方法 |
| I2C | `i2c/sal_i2c.h` | 多实例 | I2C2/3，寄存器 / 原始 Master 读写 |
| GPIO | `gpio/sal_gpio.h` | 多实例 | 任意引脚，输出控制 + EXTI 中断回调 |
| PWM | `pwm/sal_pwm.h` | 多实例 | 定时器 PWM 输出，占空比/周期可调 |
| DWT | `dwt/sal_dwt.h` | 静态+实例 | 高精度计时（微秒级），延时，代码耗时测量 |
| Flash | `flash/sal_flash.h` | 纯静态 | 内部 Flash 擦/读/写，用于掉电参数保存 |
| USB | `usb/sal_usb.h` | 单例 | USB CDC 虚拟串口 |
| Log | `log/log.h` | 宏 | Segger RTT 日志（LOGINFO / LOGWARNING / LOGERROR） |
| TaskManager | `TaskManager.hpp` | 实例 | FreeRTOS 任务封装，自带 DWT 性能统计 |

公共工具：

| 文件 | 内容 |
|---|---|
| `xStruct.hpp` | `loop_queue<T>` 定长循环队列 |
| `xTools.hpp` | `bit_locker` / `lock_guard` 轻量互斥锁，`DEBUG_DEADLOCK` 宏 |

---

## Module 层使用指南

### 通用模式

所有多实例外设遵循相同的使用流程：

```
┌──────────────────────────────────────────────────────┐
│  1. 填写 Config 结构体（handle、ID、回调等）         │
│  2. 构造 XxxInstance（自动注册到全局列表）            │
│  3. 调用实例方法发送/接收                             │
│  4. 在回调中处理接收到的数据                          │
└──────────────────────────────────────────────────────┘
```

### CAN — 电机/传感器通信

```cpp
#include "sal_can.h"

// 1. 在 module 类中持有 CAN 实例
class Motor {
    sal::CANInstance can_;

    // 2. 接收回调
    void OnCanRx(uint8_t len) {
        // 解析 can_.rx_data_ 中的数据
    }

public:
    Motor() : can_({
        .handle = &hcan1,
        .tx_id  = 0x200,
        .rx_id  = 0x201,
        .rx_cbk = [this](uint8_t len) { OnCanRx(len); },
    }) {}

    void SendCmd(uint8_t *data) {
        sal::can_msg_t msg;
        memcpy(msg.data, data, 8);
        can_.CANTransmit(msg);  // 自动进入 FIFO 排队发送
    }
};
```

### UART — 遥控器/裁判系统/视觉

```cpp
#include "sal_usart.h"

class Remote {
    sal::UARTInstance uart_;

    void OnRecv(uint8_t *buf, uint16_t len) {
        // 解析接收到的数据包
    }

public:
    Remote() : uart_({
        .handle  = &huart3,
        .rx_size = 18,                  // DBUS 一包 18 字节
        .rx_type = sal::UART_RX_DMA_IDLE,
        .rx_cbk  = [this](uint8_t *buf, uint16_t len) { OnRecv(buf, len); },
    }) {}

    void Send(uint8_t *data, uint16_t len) {
        uart_.UARTSend(data, len);
    }
};
```

### SPI — IMU (BMI088)

```cpp
#include "sal_spi.h"

class BMI088 {
    sal::SPIInstance accel_;
    sal::SPIInstance gyro_;

public:
    BMI088()
        : accel_({&hspi1, sal::SPI_XFER_BLOCK, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin}),
          gyro_ ({&hspi1, sal::SPI_XFER_BLOCK, CS1_GYRO_GPIO_Port,  CS1_GYRO_Pin})
    {}

    uint8_t ReadAccelReg(uint8_t reg) {
        return accel_.ReadReg(reg);
    }

    void ReadGyroData(uint8_t *buf) {
        gyro_.ReadRegs(0x02, buf, 6);   // 阻塞读取 6 字节
    }

    void ReadAccelDataDMA(uint8_t *buf) {
        accel_.ReadRegsDMA(0x12, buf, 6); // DMA 读取，完成后触发 rx_cbk
    }
};
```

### I2C — 磁力计/OLED

```cpp
#include "sal_i2c.h"

class IST8310 {
    sal::I2CInstance i2c_;

public:
    IST8310() : i2c_({
        .handle    = &hi2c3,
        .dev_addr  = 0x0E,
        .xfer_type = sal::I2C_XFER_BLOCK,
    }) {}

    uint8_t ReadReg(uint16_t reg) {
        uint8_t val;
        i2c_.MemRead(reg, &val, 1);
        return val;
    }
};
```

### GPIO — LED/按键

```cpp
#include "sal_gpio.h"

// 输出控制
sal::GPIOInstance led_r({LED_R_GPIO_Port, LED_R_Pin});
led_r.Reset();    // 点亮（低有效）
led_r.Set();      // 熄灭
led_r.Toggle();   // 翻转

// EXTI 中断
sal::GPIOInstance key({
    .port     = KEY_GPIO_Port,
    .pin      = KEY_Pin,
    .exti_cbk = []() { /* 按键按下处理 */ },
});
```

### PWM — 舵机/蜂鸣器

```cpp
#include "sal_pwm.h"

sal::PWMInstance servo({&htim1, TIM_CHANNEL_1});
servo.Start();
servo.SetDutyCycle(0.075f);  // 7.5% 占空比 → 舵机中位

sal::PWMInstance buzzer({&htim4, TIM_CHANNEL_3});
buzzer.Start();
buzzer.SetCompare(500);      // 直接设置 CCR 值
```

### DWT — 计时/延时

```cpp
#include "sal_dwt.h"

// 全局初始化（在 BSP init 阶段调用一次）
DWTInstance::DWTInit(168);  // 168MHz

// 获取时间轴
float now = DWTInstance::DWTGetTimeline_ms();

// 阻塞延时（不受中断影响，可用于临界区）
DWTInstance::DWTDelay(0.001f); // 延时 1ms

// 测量两次调用的间隔
DWTInstance dt_counter;
float dt = dt_counter.DWTGetDeltaT();  // 返回秒

// 代码段计时宏
TIME_ELAPSE("imu_update", {
    imu.Update();
})();
```

### Flash — 掉电保存参数

```cpp
#include "sal_flash.h"

// 存储区建议使用末尾扇区（避开程序代码区）
constexpr uint32_t PARAM_ADDR = sal::Flash::SECTOR_11_ADDR;

// 保存
float pid_params[3] = {10.0f, 0.5f, 1.0f};
sal::Flash::Erase(PARAM_ADDR, 1);                                    // 先擦除 1 个扇区
sal::Flash::Write(PARAM_ADDR, reinterpret_cast<uint32_t*>(pid_params), 3); // 写入 3 个 word

// 读取
float loaded[3];
sal::Flash::Read(PARAM_ADDR, reinterpret_cast<uint32_t*>(loaded), 3);
```

> **注意**：Flash 写入前必须先擦除，擦除以扇区为单位。Sector 0~4 存放程序代码，建议使用 Sector 5~11 存储参数。

### USB CDC — 上位机通信

```cpp
#include "sal_usb.h"

class VisionComm {
    sal::USBInstance usb_;

    void OnRecv(uint8_t *buf, uint16_t len) {
        // 处理上位机发来的数据
    }

public:
    VisionComm() : usb_({
        .rx_cbk = [this](uint8_t *buf, uint16_t len) { OnRecv(buf, len); },
    }) {}

    void Send(uint8_t *data, uint16_t len) {
        usb_.Transmit(data, len);  // 单次最大 64 字节
    }
};
```

### Log — 调试输出

```cpp
#include "log.h"

LOGINFO("motor speed = %d", speed);          // 绿色
LOGWARNING("temperature high: %d", temp);    // 黄色
LOGERROR("CAN bus error!");                  // 红色

// 浮点数需要先转换（RTT 不支持 %f）
char buf[16];
Float2Str(buf, 3.14f);
LOGINFO("pi = %s", buf);
```

### TaskManager — 任务管理

```cpp
#include "TaskManager.hpp"

TaskManager imu_task({
    .name       = "imu",
    .stack_size = 512,
    .priority   = osPriorityRealtime,
    .period_ms  = 1,
    .init_func  = []() { /* IMU 初始化 */ },
    .task_func  = []() { /* 每 1ms 执行一次 */ },
});

// 查看任务性能
// imu_task.ex_dt      → 实际任务周期 (s)
// imu_task.ex_time    → 单次执行耗时 (s)
// imu_task.ex_dt_mx   → 最大周期 (s)
// imu_task.ex_time_mx → 最大执行耗时 (s)
```

---

## 层级关系

```
┌─────────────────────────────────────────────────┐
│                   App 层                         │
│         robot.cpp / robot_task.hpp               │
├─────────────────────────────────────────────────┤
│                 Module 层                        │
│    电机、IMU、遥控器、裁判系统、视觉等           │
│    #include "sal_xxx.h" 构造实例并使用           │
├─────────────────────────────────────────────────┤
│                  SAL 层 ← 你在这里               │
│    CAN / UART / SPI / I2C / GPIO / PWM /        │
│    DWT / Flash / USB / Log / TaskManager         │
├─────────────────────────────────────────────────┤
│                  HAL 层                          │
│    STM32 HAL Driver + CubeMX 生成代码            │
└─────────────────────────────────────────────────┘
```

**关键约定**：
- Module 层通过 `#include` SAL 头文件使用硬件，**禁止直接调用 HAL 函数**
- SAL 层的回调在**中断上下文**执行，不要做耗时操作（如 printf、大量 memcpy），需要处理数据时应设置标志位，在任务中处理
- 同一个 HAL handle（如 `&hcan1`）可以被多个 SAL 实例共享（如多个电机挂在同一条 CAN 总线），SAL 内部通过 ID 分发
- Flash 操作会短暂关闭中断（HAL 实现），避免在高实时性任务中调用
