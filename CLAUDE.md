# powerful_framework 开发规范

## 构建

```bash
cmake -B build -G Ninja
cmake --build build        # -Wall -Werror, 零警告要求
```

CMake glob 自动收录 `Core/` 下所有 `.cpp/.cc/.hpp`，新增文件需重新 `cmake -B build` 配置。

## 命名规范

| 类别 | 风格 | 示例 |
|---|---|---|
| 类型 (class / struct / enum / typedef) | PascalCase | `SpiInstance`, `PidController`, `Bmi088Data` |
| 枚举成员 | UPPER_SNAKE_CASE | `BLOCK_PERIODIC`, `NO_ERROR`, `XFER_BLOCK` |
| 命名空间 | lower_snake_case | `sal`, `bmi088`, `loop_mode` |
| 函数 / 方法 | PascalCase (与 HAL 保持一致) | `SetOutput()`, `ReadReg()`, `Acquire()` |
| 成员变量 (private) | lower_snake_case + 尾随 `_` | `handle_`, `gyro_offset_`, `acc_coef_` |
| 结构体字段 (public) | lower_snake_case (无尾随 `_`) | `spi_handle`, `max_out`, `acc_cs_port` |
| 局部变量 / 参数 | lower_snake_case | `raw_data`, `whoami`, `error` |
| 全局常量 (`inline constexpr`) | UPPER_SNAKE_CASE | `PI`, `RAD_2_DEGREE`, `ACC_CHIP_ID` |
| 宏 | UPPER_SNAKE_CASE | `DEBUG_DEADLOCK`, `LOGINFO` |

### 缩写规则

缩写词仅首字母大写，其余小写，视为普通单词:

- `SpiInstance` (非 ~~SPIInstance~~)
- `CanInstance` (非 ~~CANInstance~~)
- `GpioInstance` (非 ~~GPIOInstance~~)
- `PwmInstance` (非 ~~PWMInstance~~)
- `DwtInstance` (非 ~~DWTInstance~~)
- `HttpRequest` (非 ~~HTTPRequest~~)
- `UsbDevice` (非 ~~USBDevice~~)

枚举成员和宏中的缩写照常全大写: `SPI_XFER_BLOCK`, `ACC_CHIP_ID`。

### 关键字冲突

若标识符与 C++ 关键字冲突，追加一个下划线:

```cpp
namespace namespace_ { }  // namespace 是关键字
float class_ = 0;         // class 是关键字
```

## 代码风格

- C++17, arm-none-eabi-g++, FreeRTOS
- 优先使用 `enum class` 而非无作用域 enum
- 头文件使用 `#pragma once`
- 配置用纯数据结构体 (aggregate), 不加尾随 `_`
- 私有成员用尾随 `_`
- 模块层类持有 SAL 实例指针，SAL 实例由 `new` 创建 (生命周期全局)
- 禁止手动 `delete` SAL 实例 (由静态 vector 管理)
- 常量优先用 `inline constexpr` 而非 `#define`

## 提交规范

使用 gitmoji 格式:

```
<emoji> <type>[scope]: <description>
```

示例: `✨ feat(imu): 移植 BMI088 驱动模块`

不添加 `Co-Authored-By` 或 AI 工具标识。
