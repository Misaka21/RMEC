# Referee 裁判系统

<p align='right'>powerful_framework</p>

> RoboMaster 裁判系统协议解析器，字节级状态机，传输无关设计

---

## 如果你只需要快速使用

```cpp
#include "referee/referee.hpp"
#include "app/robot_topics.hpp"

// 1. 创建解析器
referee::RefereeParser parser(
    [](uint8_t* buf, uint16_t len) {
        // 发送函数：绑定到 UART
        HAL_UART_Transmit_DMA(&huart6, buf, len);
    },
    [](const referee::RefereeData& data) {
        // ISR 回调：发布到 Topic
        referee_topic.Publish(data);
    }
);

// 2. UART 接收回调（ISR 上下文）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if (huart == &huart6) {
        parser.Parse(rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, sizeof(rx_buf));
    }
}

// 3. 读取裁判数据
void CmdTask() {
    auto reader = referee_topic.Subscribe();
    referee::RefereeData data;

    while (true) {
        if (reader->Read(data)) {
            // 血量信息
            uint16_t remain_hp = data.robot_status.remain_hp;
            uint16_t max_hp = data.robot_status.max_hp;

            // 热量信息
            uint16_t heat = data.power_heat.shooter_17mm_heat0;
            uint16_t heat_limit = data.robot_status.shooter_heat_limit;

            // 发射速度
            float speed = data.shoot_data.bullet_speed;

            // 判断是否可以发射
            if (heat < heat_limit - 100 && speed > 20.0f) {
                AllowFire();
            }
        }

        vTaskDelay(10);
    }
}
```

---

## 协议帧格式

RoboMaster 2026 V1.2.0 标准协议：

```
[SOF] [data_length] [seq] [CRC8] [cmd_id] [data(n)] [CRC16]
 1B       2B          1B     1B      2B       nB       2B

SOF = 0xA5
data_length: 数据区长度（2 字节，小端）
seq: 包序号（循环 0-255）
CRC8: 帧头校验（SOF + data_length + seq）
cmd_id: 命令码（2 字节，小端）
data: 数据区（n 字节，n = data_length）
CRC16: 整帧校验
```

### 最小帧长度

- 帧头：7 字节（SOF~CRC8）
- 最小数据：0 字节
- 最小帧：7 + 2 + 0 + 2 = 11 字节

### 最大帧长度

- 最大数据：256 字节
- 最大帧：7 + 2 + 256 + 2 = 267 字节

---

## 主要数据结构

### RefereeData

```cpp
struct RefereeData {
    GameStatus game_status;           // 0x0001 比赛状态
    RobotStatus robot_status;         // 0x0201 机器人状态
    PowerHeatData power_heat;         // 0x0202 功率热量
    ShootData shoot_data;             // 0x0207 发射数据
    // ... 其他数据
};
```

### 常用字段

```cpp
// 比赛状态 (0x0001)
struct GameStatus {
    uint8_t game_type;      // 比赛类型
    uint8_t game_progress;  // 比赛阶段 (0=未开始, 1=准备, 2=自检, 3=5s倒计时, 4=进行中, 5=结束)
    uint16_t stage_remain_time; // 剩余时间 (秒)
};

// 机器人状态 (0x0201)
struct RobotStatus {
    uint8_t robot_id;       // 机器人 ID (1-7 红方, 101-107 蓝方)
    uint8_t robot_level;    // 等级
    uint16_t remain_hp;     // 剩余血量
    uint16_t max_hp;        // 最大血量
    uint16_t shooter_heat_limit;   // 热量上限 (17mm)
    uint16_t shooter_cooling_rate; // 冷却速率
    uint8_t mains_power_gimbal_output : 1;  // 云台供电
    uint8_t mains_power_chassis_output : 1; // 底盘供电
    uint8_t mains_power_shooter_output : 1; // 发射供电
};

// 功率热量 (0x0202) - V1.2.0 版本
struct PowerHeatData {
    uint8_t reserved[6];            // 前 6 字节保留
    uint16_t chassis_power_buffer;  // 缓冲能量 (J)
    uint16_t shooter_17mm_heat0;    // 17mm 枪口热量 0
    uint16_t shooter_17mm_heat1;    // 17mm 枪口热量 1
    uint16_t shooter_42mm_heat;     // 42mm 枪口热量
};

// 发射数据 (0x0207)
struct ShootData {
    uint8_t bullet_type;    // 弹丸类型 (1=17mm, 2=42mm)
    uint8_t shooter_id;     // 发射机构 ID
    uint8_t bullet_freq;    // 弹频 (发/秒)
    float bullet_speed;     // 弹速 (m/s)
};
```

### Robot ID 对照表

| ID | 红方 | 蓝方 |
|----|------|------|
| 1/101 | 英雄 | 英雄 |
| 2/102 | 工程 | 工程 |
| 3/103 | 步兵 3 | 步兵 3 |
| 4/104 | 步兵 4 | 步兵 4 |
| 5/105 | 步兵 5 | 步兵 5 |
| 6/106 | 空中 | 空中 |
| 7/107 | 哨兵 | 哨兵 |

---

## 发送数据到裁判系统

### 发送原始帧

```cpp
// 构造数据
uint8_t payload[] = {0x01, 0x02, 0x03};
parser.Send(0x0301, payload, sizeof(payload));
```

### 发送交互数据 (0x0301)

```cpp
// 发送自定义数据到选手端
uint8_t content[] = {0x01, 0x02};
parser.SendInteraction(
    SUB_CMD_CUSTOM_CONTROLLER,  // 子命令: 自定义控制器
    0,                          // receiver_id: 0 = 自动推算
    content,                    // 内容
    sizeof(content)            // 长度
);
```

### 发送哨兵自主决策指令

```cpp
// bit 编码控制哨兵行为
uint32_t sentry_cmd = 0x00000001;  // 详见规则手册
parser.SendSentryDecision(sentry_cmd);
```

---

## 双链路设计

裁判系统有两条独立链路：

### 常规链路 (USART6, 115200)

- 接收：比赛状态、机器人状态、血量、热量、发射数据等
- 发送：交互数据、选手端数据
- 所有板子都可以连接

### 图传链路 (USART1, 921600)

- 接收：图传专用数据
- 发送：UI 绘制数据
- 仅云台板/一体板连接

```cpp
// 常规链路
RefereeParser referee_parser(
    [](uint8_t* buf, uint16_t len) { HAL_UART_Transmit_DMA(&huart6, buf, len); },
    [](const RefereeData& d) { referee_topic.Publish(d); }
);

// 图传链路（如有）
RefereeParser video_link_parser(
    [](uint8_t* buf, uint16_t len) { HAL_UART_Transmit_DMA(&huart1, buf, len); },
    nullptr  // 不需要发布
);
```

---

## 板间门控

```cpp
// robot_def.hpp
#define BOARD_TYPE ONE_BOARD  // 或 CHASSIS_BOARD / GIMBAL_BOARD

// 常规链路：所有板子都可以使用
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    #define REFEREE_ENABLE 1
#endif

// 图传链路：只有云台板使用
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    #define VIDEO_LINK_ENABLE 1
#endif
```

---

## 完整示例：裁判任务

```cpp
// referee_task.cpp
#include "referee/referee.hpp"
#include "daemon/daemon.hpp"
#include "app/robot_topics.hpp"

using namespace referee;

static RefereeParser* parser = nullptr;
static DaemonInstance* daemon = nullptr;

void RefereeTaskStart() {
    // 创建解析器
    parser = new RefereeParser(
        [](uint8_t* buf, uint16_t len) {
            HAL_UART_Transmit_DMA(&huart6, buf, len);
        },
        [](const RefereeData& data) {
            referee_topic.Publish(data);
        }
    );

    // 创建守护
    daemon = new DaemonInstance({
        .timeout_ticks = 100,  // 1s 超时（裁判系统 10Hz）
        .on_offline = [](void*) { LOGWARN("Referee offline!"); },
        .owner = parser
    });

    // 启动 UART 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, sizeof(rx_buf));

    // 喂狗任务
    xTaskCreate([](void*) {
        while (true) {
            // 只要有数据解析成功，就喂狗
            // 在 Parse 回调中自动发布到 Topic
            // 这里通过检查 topic 更新或 parser 内部状态喂狗
            daemon->Reload();
            vTaskDelay(50);
        }
    }, "RefereeDaemon", 256, nullptr, 2, nullptr);
}

// UART 中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if (huart == &huart6 && parser) {
        parser->Parse(rx_buf, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, sizeof(rx_buf));
    }
}

// 辅助函数
uint8_t GetRobotId() {
    return parser ? parser->RobotId() : 0;
}

bool IsRedTeam() {
    return GetRobotId() > 0 && GetRobotId() < 100;
}

bool IsBlueTeam() {
    return GetRobotId() > 100;
}
```

---

## 常用查询函数

```cpp
// 获取本机 robot_id
uint8_t RobotId() const;

// 获取本机对应的选手端 ID (robot_id + 0x0100)
uint16_t ClientId() const;
// 红方: 0x0101~0x0107
// 蓝方: 0x0165~0x016B
```

---

## 与 basic_framework 的区别

| 特性 | basic_framework | powerful_framework |
|------|-----------------|-------------------|
| 解析 | 回调+全局变量 | 状态机+Topic |
| 发送 | 直接 UART | 函数指针注入 |
| 协议版本 | 2025 | 2026 V1.2.0 |
| 数据存储 | 分散变量 | RefereeData 结构体 |
| 双链路 | 需手动实现 | 两个独立实例 |

---

## 注意事项

1. **协议版本**: V1.2.0 的 0x0202 PowerHeat 前 6 字节为保留，注意兼容性

2. **双链路波特率**: 常规 115200，图传 921600，不要混淆

3. **发送长度限制**: 0x0301 交互数据内容最大 112 字节

4. **接收缓冲**: UART 接收缓冲区建议 256 字节以上

5. **板间门控**: 确保 CHASSIS/GIMBAL/ONE_BOARD 宏定义正确
