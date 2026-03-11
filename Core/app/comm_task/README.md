# comm_task

双板通信任务, 100 Hz FreeRTOS 任务。

## 工作流程

### 初始化

1. 根据 `robot_def.hpp` 中的 `COMM_CAN_HANDLE`, `COMM_BASE_TX_ID`, `COMM_BASE_RX_ID` 创建 `CanComm` 实例
2. 配置 Daemon 超时 (100 tick @ 100Hz = 1s)

### 循环 (100 Hz)

1. 组装 `BoardCommTxData` 并通过 `comm->Send()` 发送
2. 调用 `board_comm_topic.Publish(comm->Recv())` 发布接收数据

### 条件编译

| 板型 | 行为 |
|---|---|
| `ONE_BOARD` | `CommTaskStart()` 为空函数, 不创建任务 |
| `GIMBAL_BOARD` | 运行完整通信任务 |
| `CHASSIS_BOARD` | 运行完整通信任务 |

## 数据类型

`BoardCommTxData` 和 `BoardCommRxData` 定义在 `robot_topics.hpp`, 当前为 placeholder:

```cpp
#pragma pack(1)
struct BoardCommTxData {
    uint8_t placeholder[8];  // 用户填入实际字段
};
struct BoardCommRxData {
    uint8_t placeholder[8];  // 用户填入实际字段
};
#pragma pack()
```

使用时按实际通信需求替换字段。注意两板的 Tx/Rx 互为对称: A 板的 TxData 是 B 板的 RxData。

## 代码结构

| 文件 | 说明 |
|---|---|
| `comm_task.hpp` | `CommTaskStart()` 声明 |
| `comm_task.cpp` | TaskManager + CanComm 实例化 |

## 配置 (robot_def.hpp)

```cpp
#define COMM_CAN_HANDLE  hcan1
inline constexpr uint16_t COMM_BASE_TX_ID = 0x100;
inline constexpr uint16_t COMM_BASE_RX_ID = 0x110;
```

双板配置时, A 板的 `BASE_TX_ID` 应等于 B 板的 `BASE_RX_ID`, 反之亦然。
