#include "dt7_protocol.hpp"
#include <cstring>

namespace remote {

void Dt7Protocol::Decode(const uint8_t* buf, Data& data, const Data& prev) {
    // ---- 摇杆通道 (11-bit, 偏移 1024) ----
    data.ch_r_x = static_cast<int16_t>( (buf[0] | (buf[1] << 8))                          & 0x07FF) - CH_OFFSET;
    data.ch_r_y = static_cast<int16_t>(((buf[1] >> 3) | (buf[2] << 5))                    & 0x07FF) - CH_OFFSET;
    data.ch_l_x = static_cast<int16_t>(((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10))  & 0x07FF) - CH_OFFSET;
    data.ch_l_y = static_cast<int16_t>(((buf[4] >> 1) | (buf[5] << 7))                    & 0x07FF) - CH_OFFSET;
    data.dial   = static_cast<int16_t>( (buf[16] | (buf[17] << 8))                        & 0x07FF) - CH_OFFSET;

    // ---- 拨杆 (byte 5 高 4 位) ----
    data.sw_r = static_cast<SwitchPos>((buf[5] >> 4) & 0x03);
    data.sw_l = static_cast<SwitchPos>(((buf[5] >> 4) & 0x0C) >> 2);

    // ---- 鼠标 (int16 小端) ----
    data.mouse_x = static_cast<int16_t>(buf[6]  | (buf[7]  << 8));
    data.mouse_y = static_cast<int16_t>(buf[8]  | (buf[9]  << 8));
    data.mouse_l = buf[12];
    data.mouse_r = buf[13];

    // ---- 键盘 ----
    data.keys = static_cast<uint16_t>(buf[14] | (buf[15] << 8));

    // 修饰键组合: 仅在对应修饰键按下时保留按键位
    data.keys_with_ctrl  = (data.keys & (1u << KEY_CTRL))  ? data.keys : 0;
    data.keys_with_shift = (data.keys & (1u << KEY_SHIFT)) ? data.keys : 0;

    // ---- 按键上升沿计数 ----
    // 继承上一帧的累计计数
    std::memcpy(data.key_count, prev.key_count, sizeof(data.key_count));

    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (i == KEY_SHIFT || i == KEY_CTRL) continue;

        uint16_t mask = 1u << i;

        // 无修饰键的纯按下: 上升沿 + 当前帧不含 Ctrl/Shift 组合
        if ((data.keys & mask) && !(prev.keys & mask) &&
            !(data.keys_with_ctrl & mask) && !(data.keys_with_shift & mask)) {
            data.key_count[KEY_PRESS][i]++;
        }

        // Ctrl + key 上升沿
        if ((data.keys_with_ctrl & mask) && !(prev.keys_with_ctrl & mask)) {
            data.key_count[KEY_PRESS_WITH_CTRL][i]++;
        }

        // Shift + key 上升沿
        if ((data.keys_with_shift & mask) && !(prev.keys_with_shift & mask)) {
            data.key_count[KEY_PRESS_WITH_SHIFT][i]++;
        }
    }

    // ---- 通道矫正: 超出有效范围则归零 ----
    auto rectify = [](int16_t& ch) {
        if (ch > CH_MAX || ch < -CH_MAX) ch = 0;
    };
    rectify(data.ch_r_x);
    rectify(data.ch_r_y);
    rectify(data.ch_l_x);
    rectify(data.ch_l_y);
    rectify(data.dial);
}

void Dt7Protocol::Reset(Data& data) {
    std::memset(&data, 0, sizeof(Data));
}

} // namespace remote
