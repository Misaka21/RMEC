# Vision 视觉通信

<p align='right'>powerful_framework</p>

> 视觉识别数据通信模块，32B 固定包协议，CRC16-CCITT 校验

---

## 如果你只需要快速使用

```cpp
#include "vision/vision_comm.hpp"
#include "vision/vision_protocol.hpp"
#include "app/robot_topics.hpp"

// 1. 创建视觉通信实例
vision::VisionComm vision_comm(
    [](uint8_t* buf, uint16_t len) {
        // 发送函数：绑定到 UART
        HAL_UART_Transmit_DMA(&huart1, buf, len);
    },
    [](const vision::VisionRxData& data) {
        // ISR 回调：发布到 Topic
        vision_topic.Publish(data);
    }
);

// 2. UART 接收回调（ISR 上下文）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        vision_comm.OnReceive(rx_buf, len);
    }
}

// 3. 发送数据（Task 上下文）
void VisionTask() {
    auto ins_reader = ins_topic.Subscribe();

    while (true) {
        // 读取 INS 数据
        InsData ins;
        if (ins_reader->Read(ins)) {
            // 填充发送数据
            vision::VisionTxData tx;
            tx.mode = static_cast<uint8_t>(AimMode::AUTO_AIM);
            tx.aiming_lock = is_locked;
            tx.bullet_speed = 25.0f;
            tx.yaw = ins.yaw;      // rad
            tx.pitch = ins.pitch;  // rad
            tx.roll = ins.roll;    // rad
            tx.enemy_color = static_cast<uint8_t>(EnemyColor::RED);

            // 发送
            vision_comm.Send(tx);
        }

        vTaskDelay(10);  // 100Hz
    }
}

// 4. 读取接收数据
void CmdTask() {
    auto vision_reader = vision_topic.Subscribe();

    while (true) {
        vision::VisionRxData rx;
        if (vision_reader->Read(rx)) {
            // 使用视觉目标数据
            float target_yaw = rx.yaw;
            float target_pitch = rx.pitch;

            if (rx.control & 0x01) {
                // 自动瞄准模式
                SetGimbalTarget(target_yaw, target_pitch);
            }

            if (rx.shoot & 0x01) {
                // 发射命令
                Fire();
            }
        }

        vTaskDelay(1);
    }
}
```

---

## 通信协议

### 帧格式（32 字节固定长度）

```
字节:   0      1       2       3-6       7-10      11-14     15      16-30      31
      [SOF] [control][shoot] [yaw]    [pitch]    [roll]   [color] [reserved] [EOF]
      0xFF   1B      1B      4B(float) 4B(float) 4B(float)  1B       15B       0x0D
```

### CRC16 范围

- 校验范围：字节 1~28（共 28 字节 payload）
- 存储位置：字节 29~30（小端序）

### 接收数据结构 (MCU ← Vision)

```cpp
struct VisionRxData {
    uint8_t control;      // 控制字节
    uint8_t shoot;        // 发射控制
    float yaw;            // 目标 yaw (rad)
    float pitch;          // 目标 pitch (rad)
    uint8_t reserved[18]; // 保留
};
```

### 发送数据结构 (MCU → Vision)

```cpp
struct VisionTxData {
    uint8_t mode;         // 模式
    uint8_t aiming_lock;  // 瞄准锁定
    float bullet_speed;   // 弹速 (m/s)
    float yaw;            // 当前 yaw (rad)
    float pitch;          // 当前 pitch (rad)
    float roll;           // 当前 roll (rad)
    uint8_t enemy_color;  // 敌方颜色
    uint8_t reserved[13]; // 保留
};
```

---

## 枚举定义

### 瞄准模式

```cpp
enum class AimMode : uint8_t {
    NO_AIM    = 0,  // 不瞄准
    AUTO_AIM  = 1,  // 自动瞄准
    AUTO_SHOOT = 2, // 自动发射
};
```

### 敌方颜色

```cpp
enum class EnemyColor : uint8_t {
    RED  = 0,
    BLUE = 1,
};
```

---

## 传输无关设计

VisionComm **不持有 UART 实例**，通过函数指针注入发送能力：

```cpp
// 使用 UART
VisionComm comm(
    [](uint8_t* buf, uint16_t len) {
        HAL_UART_Transmit_DMA(&huart1, buf, len);
    }
);

// 使用 USB CDC
VisionComm comm(
    [](uint8_t* buf, uint16_t len) {
        CDC_Transmit_FS(buf, len);
    }
);
```

---

## SeqLock 读取

```cpp
// 方式 1：直接读取（可能失败）
vision::VisionRxData rx;
if (vision_comm.Recv(rx)) {
    // 读取成功
}

// 方式 2：通过 Topic（推荐）
auto reader = vision_topic.Subscribe();
if (reader->Read(rx)) {
    // 读取成功
}
```

---

## 完整示例：视觉任务

```cpp
// vision_task.cpp
#include "vision/vision_comm.hpp"
#include "vision/vision_protocol.hpp"
#include "daemon/daemon.hpp"
#include "app/robot_topics.hpp"

using namespace vision;

static VisionComm* vision = nullptr;
static DaemonInstance* vision_daemon = nullptr;

// 全局状态（供 cmd_task 设置）
static AimMode current_mode = AimMode::NO_AIM;
static uint8_t aiming_lock = 0;

void VisionTaskStart() {
    // 创建视觉通信
    vision = new VisionComm(
        [](uint8_t* buf, uint16_t len) {
            HAL_UART_Transmit_DMA(&huart1, buf, len);
        },
        [](const VisionRxData& data) {
            vision_topic.Publish(data);
        }
    );

    // 创建守护
    vision_daemon = new DaemonInstance({
        .timeout_ticks = 50,  // 500ms
        .on_offline = [](void*) { LOGWARN("Vision offline!"); },
        .owner = vision
    });

    // 接收回调（在 UART ISR 中调用）
    // 收到数据后会自动调用 OnPublish，然后在这里喂狗

    xTaskCreate([](void*) {
        auto ins_reader = ins_topic.Subscribe();
        VisionRxData rx;

        while (true) {
            // 喂狗检查
            if (vision->Recv(rx)) {
                vision_daemon->Reload();
            }

            // 发送（100Hz）
            InsData ins;
            if (ins_reader->Read(ins)) {
                VisionTxData tx;
                tx.mode = static_cast<uint8_t>(current_mode);
                tx.aiming_lock = aiming_lock;
                tx.bullet_speed = GetBulletSpeed();
                tx.yaw = ins.yaw;
                tx.pitch = ins.pitch;
                tx.roll = ins.roll;
                tx.enemy_color = GetEnemyColor();

                vision->Send(tx);
            }

            vTaskDelay(10);
        }
    }, "VisionTask", 512, nullptr, 2, nullptr);
}

// 供 cmd_task 调用
void VisionSetMode(AimMode mode) {
    current_mode = mode;
}

void VisionSetAimingLock(bool lock) {
    aiming_lock = lock ? 1 : 0;
}

bool VisionIsOnline() {
    return vision_daemon && vision_daemon->IsOnline();
}
```

---

## CRC16-CCITT 校验

```cpp
// 多项式: x^16 + x^12 + x^5 + 1 (0x8408)
// 初始值: 0xFFFF

uint16_t CalcCRC16(const uint8_t* data, uint16_t len);
bool VerifyCRC16(const uint8_t* frame);  // 校验 buf[1..28]
void AppendCRC16(uint8_t* frame);        // 写入 buf[29..30]
```

---

## 与 basic_framework 的区别

| 特性 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| 协议 | 自定义不定长 | 32B 固定长度 |
| 校验 | CRC16 | CRC16-CCITT |
| 传输 | 直接操作 UART | 函数指针注入 |
| 读取 | 全局变量 | SeqLock + Topic |
| 解耦 | 紧耦合 | 传输无关 |

---

## 注意事项

1. **角度单位**: 全部使用**弧度制**（rad），不是角度制

2. **字节序**: float 采用小端序（ARM 默认）

3. **CRC 失败**: CRC 校验失败的数据会自动丢弃

4. **单写者**: VisionComm 是 vision_topic 的唯一写者，daemon 只喂狗

5. **发送频率**: 建议 100Hz，过高会占用过多带宽
