# remote

遥控器模块, 使用模板策略 `Remote<Protocol>` 实现零虚函数多态。

## 总览

Remote 是 UART 中断回调驱动的数据源:

```
UART DMA ISR → Remote::OnReceive()
             → Protocol::Decode()
             → publish_(data_)        ← App 层注入的回调
```

模块不持有 Topic, 通过构造时注入的 `PublishFn` 回调将解析后的数据交给 App 层发布。

## 代码结构

```
remote/
├── remote.hpp                       # Remote<Protocol> 模板主类
└── protocol/
    ├── dt7_data.hpp                 # Dt7Data 数据结构 (轻量, 供 Topic)
    ├── dt7_protocol.hpp/cpp         # DT7/DR16 DBUS 协议解析
    └── (未来: sbus_protocol.hpp)    # 扩展其他协议
```

## Protocol 接口契约

编译期 `static_assert` 检查, Protocol 需满足:

```cpp
struct SomeProtocol {
    static constexpr uint16_t FRAME_SIZE = ...;   // 帧长度
    using Data = SomeData;                         // trivially copyable 数据类型
    static void Decode(const uint8_t* buf, Data& curr, const Data& prev);
    static void Reset(Data& data);                 // 离线清零
};
```

## Remote\<Protocol\>

### 外部接口

```cpp
using PublishFn = void(*)(const Data&);

Remote(const RemoteConfig& cfg, PublishFn publish);
bool IsOnline() const;      // 在线判断
void Tick();                 // 离线计数 + 超时清零 + UART 重启
```

### 离线检测

- `OnReceive()` (ISR) 每次收到有效帧, 重置 `offline_cnt_`
- `Tick()` 每次调用递增计数, 超过 `offline_threshold` 后:
  - 调用 `Protocol::Reset()` 清零数据
  - 调用 `publish_()` 发布清零数据 (安全状态)
  - 重启 UART DMA 接收

## DT7/DR16 协议

DBUS 协议, 100000 baud, 8E1, 18 字节帧, ~70 Hz:

| 字段 | 范围 | 说明 |
|---|---|---|
| ch_r_x/y, ch_l_x/y | [-660, 660] | 摇杆通道 |
| dial | [-660, 660] | 侧面拨轮 |
| sw_l, sw_r | UP/MID/DOWN | 三段拨杆 |
| mouse_x/y | int16 | 鼠标速度 |
| mouse_l/r | 0/1 | 鼠标按键 |
| keys | 16-bit | 键盘位域 (W/S/D/A/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B) |

### 键盘边沿检测

`key_count[mode][key]` 记录每个按键的上升沿次数 (含 Ctrl/Shift 修饰):

```cpp
data.KeyPressed(KEY_W)                        // W 是否按下
data.KeyToggled(KEY_PRESS_WITH_CTRL, KEY_Q)   // Ctrl+Q 切换状态
```

## 使用范例

```cpp
// App 层初始化
remote::RemoteConfig cfg{};
cfg.uart_handle = &huart3;

auto* rc = new remote::Remote<remote::Dt7Protocol>(cfg, [](const remote::Dt7Data& d) {
    remote_topic.Publish(d);
});

// 消费者 (cmd_task)
auto* reader = remote_topic.Subscribe();
remote::Dt7Data rc_data;
if (reader->Read(rc_data)) {
    // 映射到底盘/云台/发射命令...
}
```
