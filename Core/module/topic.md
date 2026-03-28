# Topic 发布订阅

<p align='right'>powerful_framework</p>

> 单生产者多消费者无锁队列，SeqLock 实现，ISR 安全

---

## 如果你只需要快速使用

```cpp
#include "module/topic.hpp"

// 1. 定义数据结构（必须是 trivially copyable）
struct ImuData {
    float yaw, pitch, roll;
    float gyro[3];
    float acc[3];
};

// 2. 创建 Topic（全局/inline，多文件可见）
inline Topic<ImuData> imu_topic;

// ============ 生产者（单写者） ============
void ProducerTask() {
    while (true) {
        ImuData data;
        data.yaw = ReadYaw();
        data.pitch = ReadPitch();
        // ...

        imu_topic.Publish(data);  // ISR 安全
        vTaskDelay(1);
    }
}

// ============ 消费者（多读者） ============
void ConsumerTask() {
    // 订阅获取独立 reader
    auto reader = imu_topic.Subscribe();

    while (true) {
        ImuData data;
        if (reader->Read(data)) {  // 非阻塞，可能失败
            // 处理新数据
            Process(data);
        }
        vTaskDelay(1);
    }
}
```

---

## 核心设计

### SeqLock 机制

```
写者（单）：                 读者（多）：
    │                          │
    ▼                          ▼
++seq (奇) ─────┐         读 seq (s1)
data = msg      │         if (s1 奇) return false
++seq (偶) ◄────┘         拷贝 data
                          读 seq (s2)
                          if (s1 != s2) return false
                          成功！
```

### 特性

| 特性 | 说明 |
|------|------|
| 单生产者 | 仅一个任务/ISR 调用 Publish() |
| 多消费者 | 每个消费者独立 Subscribe() 获取 reader |
| 无锁 | 无互斥锁，无优先级反转 |
| ISR 安全 | Read() 和 Publish() 均可在 ISR 调用 |
| 最新值 | 只保留最新数据，旧数据覆盖 |

---

## API 详解

### Topic<T, MaxSubs>

```cpp
template <typename T, uint8_t MaxSubs = 8>
class Topic {
public:
    // 发布数据（单写者）
    void Publish(const T& msg);

    // 订阅获取 reader（初始化时调用）
    TopicReader<T>* Subscribe();

    // 读取最新值（宽松一致性，总是成功）
    const T& Latest() const;
};
```

### TopicReader<T>

```cpp
template <typename T>
class TopicReader {
public:
    // 读取新数据（严格一致性，可能失败）
    bool Read(T& out) const;

    // 检查是否有新数据
    bool HasNew() const;

    // 获取最后读取的序号
    uint32_t LastSeq() const;
};
```

---

## 完整示例：IMU 数据广播

```cpp
// robot_topics.hpp
#pragma once
#include "module/topic.hpp"
#include "imu/ins_data.hpp"
#include "vision/vision_data.hpp"
#include "referee/referee_def.hpp"

// 定义 Topic（inline，可跨文件使用）
inline Topic<InsData> ins_topic;           // IMU 姿态数据
inline Topic<VisionRxData> vision_topic;   // 视觉识别结果
inline Topic<RefereeData> referee_topic;   // 裁判系统数据
```

```cpp
// ins_task.cpp（生产者）
#include "robot_topics.hpp"

void InsTask() {
    Bmi088 imu(cfg);
    Ins ins;
    // ... 初始化

    while (true) {
        Bmi088Data data;
        if (imu.Acquire(data)) {
            InsData result = ins.Update(data, dt);
            ins_topic.Publish(result);  // 1kHz 发布
        }
        vTaskDelay(1);
    }
}
```

```cpp
// motor_task.cpp（消费者 1）
#include "robot_topics.hpp"

void MotorTask() {
    auto ins_reader = ins_topic.Subscribe();

    while (true) {
        InsData ins;
        if (ins_reader->Read(ins)) {
            // 使用 INS 数据控制云台
            gimbal_yaw.SetRef(ins.yaw);
            gimbal_pitch.SetRef(ins.pitch);
        }
        vTaskDelay(1);
    }
}
```

```cpp
// vision_task.cpp（消费者 2）
#include "robot_topics.hpp"

void VisionTask() {
    auto ins_reader = ins_topic.Subscribe();

    while (true) {
        InsData ins;
        if (ins_reader->Read(ins)) {
            // 发送姿态给视觉
            VisionTxData tx;
            tx.yaw = ins.yaw;
            tx.pitch = ins.pitch;
            tx.roll = ins.roll;
            vision_comm.Send(tx);
        }
        vTaskDelay(10);
    }
}
```

---

## 与 FreeRTOS 队列对比

| 特性 | Topic (SeqLock) | FreeRTOS Queue |
|------|-----------------|----------------|
| 语义 | Latest-value | FIFO |
| 数据保存 | 只保留最新 | 可配置长度 |
| 读取方式 | 多读者独立追踪 | 读出即移除 |
| ISR 安全 | 读/写均可 | 需 FromISR 版本 |
| 阻塞 | 无 | 可阻塞等待 |
| 内存 | 静态分配 | 动态/静态可选 |
| 适用场景 | 传感器广播 | 命令传递 |

### 选型建议

- **用 Topic**: 传感器数据（IMU、视觉、裁判），多个消费者同时需要最新值
- **用 Queue**: 任务间命令传递（需要保序、不丢失）

---

## 注意事项

1. **Trivially Copyable**: 数据类型必须平凡可复制（无指针、无动态内存）

2. **单写者约束**: 只能有一个任务调用 Publish，否则会数据错乱

3. **Subscribe 时机**: 只能在初始化时 Subscribe，运行时不能新增订阅者

4. **Read 可能失败**: 读取时如果正在写入，返回 false，消费者应处理失败

5. **Lastest() 慎用**: 返回引用，可能读到中间态，仅用于调试/显示

---

## 面试问题速查

详见 `topic_readme.md`，包含：
- SeqLock 原理
- 为什么不用 std::atomic
- 内存屏障作用
- 单生产者约束
- 与队列的对比
