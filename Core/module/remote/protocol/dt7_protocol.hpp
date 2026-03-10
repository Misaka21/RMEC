#pragma once

#include "dt7_data.hpp"

namespace remote {

/// DT7/DR16 DBUS 协议策略类
///
/// 满足 Remote<Protocol> 的编译期契约:
///   - FRAME_SIZE : uint16_t 常量
///   - Data       : trivially copyable 数据类型
///   - Decode     : static void (const uint8_t*, Data&, const Data&)
///   - Reset      : static void (Data&)
struct Dt7Protocol {
    static constexpr uint16_t FRAME_SIZE = 18;
    using Data = Dt7Data;

    /// 解码 18 字节 DBUS 帧, 使用 prev 做上升沿检测
    static void Decode(const uint8_t* buf, Data& data, const Data& prev);

    /// 离线时清零全部字段
    static void Reset(Data& data);
};

} // namespace remote
