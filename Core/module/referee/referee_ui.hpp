#pragma once

#include "referee.hpp"
#include "referee_protocol.hpp"

#include <cstdint>
#include <cstring>

namespace referee {

// ======================== 图形操作/类型/颜色枚举 ========================

enum class GraphicOp : uint8_t {
    NONE   = 0,
    ADD    = 1,
    MODIFY = 2,
    DELETE = 3,
};

enum class GraphicType : uint8_t {
    LINE    = 0,
    RECT    = 1,
    CIRCLE  = 2,
    ELLIPSE = 3,
    ARC     = 4,
    FLOAT   = 5,
    INT     = 6,
    STRING  = 7,
};

enum class GraphicColor : uint8_t {
    TEAM   = 0,  // 己方颜色 (红/蓝)
    YELLOW = 1,
    GREEN  = 2,
    ORANGE = 3,
    PURPLE = 4,
    PINK   = 5,
    CYAN   = 6,
    BLACK  = 7,
    WHITE  = 8,
};

// ======================== RefereeUi ========================
/// UI 绘图领域逻辑, 负责图形元素构建 + 批量发送
///
/// 使用方式:
///   RefereeUi ui(parser);
///   ui.Add(RefereeUi::Line("L01", 0, GraphicColor::GREEN, 2, 100, 100, 200, 200));
///   ui.Add(RefereeUi::Circle("C01", 0, GraphicColor::YELLOW, 2, 960, 540, 50));
///   ui.Flush();  // 自动选择 0x0102 (2个图形)
///
///   ui.DrawString("S01", 1, GraphicColor::WHITE, 20, 2, 100, 800, "HELLO", 5);
///   ui.DeleteLayer(0);

class RefereeUi {
public:
    explicit RefereeUi(RefereeParser& parser) : parser_(parser) {}

    // ======================== 图形工厂 (static, 纯函数) ========================

    static WireGraphicData Line(const char name[3], uint8_t layer, GraphicColor color,
                                uint16_t width, uint16_t x1, uint16_t y1,
                                uint16_t x2, uint16_t y2,
                                GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::LINE);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.width        = width;
        g.start_x      = x1;
        g.start_y      = y1;
        g.details_d    = x2;
        g.details_e    = y2;
        return g;
    }

    static WireGraphicData Rect(const char name[3], uint8_t layer, GraphicColor color,
                                uint16_t width, uint16_t x1, uint16_t y1,
                                uint16_t x2, uint16_t y2,
                                GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::RECT);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.width        = width;
        g.start_x      = x1;
        g.start_y      = y1;
        g.details_d    = x2;
        g.details_e    = y2;
        return g;
    }

    static WireGraphicData Circle(const char name[3], uint8_t layer, GraphicColor color,
                                  uint16_t width, uint16_t cx, uint16_t cy,
                                  uint16_t radius,
                                  GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::CIRCLE);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.width        = width;
        g.start_x      = cx;
        g.start_y      = cy;
        g.details_c    = radius;
        return g;
    }

    static WireGraphicData Ellipse(const char name[3], uint8_t layer, GraphicColor color,
                                   uint16_t width, uint16_t cx, uint16_t cy,
                                   uint16_t rx, uint16_t ry,
                                   GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::ELLIPSE);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.width        = width;
        g.start_x      = cx;
        g.start_y      = cy;
        g.details_d    = rx;
        g.details_e    = ry;
        return g;
    }

    static WireGraphicData Arc(const char name[3], uint8_t layer, GraphicColor color,
                               uint16_t width, uint16_t cx, uint16_t cy,
                               uint16_t start_angle, uint16_t end_angle,
                               uint16_t rx, uint16_t ry,
                               GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::ARC);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.details_a    = start_angle;
        g.details_b    = end_angle;
        g.width        = width;
        g.start_x      = cx;
        g.start_y      = cy;
        g.details_d    = rx;
        g.details_e    = ry;
        return g;
    }

    /// 浮点数图形: 实际显示值 = value, 内部乘以 1000 编码
    static WireGraphicData Float(const char name[3], uint8_t layer, GraphicColor color,
                                 uint16_t font_size, uint16_t width,
                                 uint16_t x, uint16_t y, float value,
                                 GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::FLOAT);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.details_a    = font_size;
        g.width        = width;
        g.start_x      = x;
        g.start_y      = y;
        // 浮点数编码: 将 value×1000 存入 details_c/d/e (共 32 bit)
        auto val = static_cast<int32_t>(value * 1000.0f);
        uint32_t u;
        std::memcpy(&u, &val, sizeof(u));
        g.details_c = u & 0x3FF;         // bit 0-9
        g.details_d = (u >> 10) & 0x7FF; // bit 10-20
        g.details_e = (u >> 21) & 0x7FF; // bit 21-31
        return g;
    }

    /// 整型数图形
    static WireGraphicData Int(const char name[3], uint8_t layer, GraphicColor color,
                               uint16_t font_size, uint16_t width,
                               uint16_t x, uint16_t y, int32_t value,
                               GraphicOp op = GraphicOp::ADD) {
        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::INT);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.details_a    = font_size;
        g.width        = width;
        g.start_x      = x;
        g.start_y      = y;
        uint32_t u;
        std::memcpy(&u, &value, sizeof(u));
        g.details_c = u & 0x3FF;
        g.details_d = (u >> 10) & 0x7FF;
        g.details_e = (u >> 21) & 0x7FF;
        return g;
    }

    // ======================== 批量队列 ========================

    /// 入队一个图形元素 (最多 7 个, 超出自动 Flush)
    void Add(const WireGraphicData& g) {
        if (count_ >= MAX_BATCH) {
            Flush();
        }
        buffer_[count_++] = g;
    }

    /// 发送已入队的图形 (自动选择最优 sub_cmd_id)
    void Flush() {
        if (count_ == 0) return;

        uint16_t sub_cmd_id;
        uint16_t n;
        if (count_ == 1)      { sub_cmd_id = SUB_CMD_DRAW_1; n = 1; }
        else if (count_ == 2) { sub_cmd_id = SUB_CMD_DRAW_2; n = 2; }
        else if (count_ <= 5) { sub_cmd_id = SUB_CMD_DRAW_5; n = 5; }
        else                  { sub_cmd_id = SUB_CMD_DRAW_7; n = 7; }

        // 打包: n 个 WireGraphicData (每个 15B)
        uint8_t payload[15 * 7]{};
        uint16_t payload_len = static_cast<uint16_t>(n * sizeof(WireGraphicData));
        for (uint16_t i = 0; i < count_; ++i) {
            std::memcpy(payload + i * sizeof(WireGraphicData),
                        &buffer_[i], sizeof(WireGraphicData));
        }
        // 未填满的槽位保持零 (空操作图形)

        parser_.SendInteraction(sub_cmd_id, 0, payload, payload_len);
        count_ = 0;
    }

    // ======================== 字符串 ========================

    /// 绘制字符串 (单独发送, 不走批量队列)
    /// @param str 字符内容
    /// @param len 字符长度 (最大 30)
    void DrawString(const char name[3], uint8_t layer, GraphicColor color,
                    uint16_t font_size, uint16_t width,
                    uint16_t x, uint16_t y,
                    const char* str, uint8_t len,
                    GraphicOp op = GraphicOp::ADD) {
        // 字符串 payload = 15B graphic_config + 30B char
        uint8_t payload[45]{};

        WireGraphicData g{};
        SetName(g, name);
        g.operate_type = static_cast<uint32_t>(op);
        g.figure_type  = static_cast<uint32_t>(GraphicType::STRING);
        g.layer        = layer;
        g.color        = static_cast<uint32_t>(color);
        g.details_a    = font_size;
        g.details_b    = (len > 30) ? 30 : len;
        g.width        = width;
        g.start_x      = x;
        g.start_y      = y;

        std::memcpy(payload, &g, sizeof(WireGraphicData));

        uint8_t copy_len = (len > 30) ? 30 : len;
        std::memcpy(payload + sizeof(WireGraphicData), str, copy_len);

        parser_.SendInteraction(SUB_CMD_DRAW_STRING, 0, payload, 45);
    }

    // ======================== 图层操作 ========================

    void DeleteLayer(uint8_t layer) {
        WireGraphicDelete del{};
        del.delete_type = 1;
        del.layer       = layer;
        parser_.SendInteraction(SUB_CMD_DELETE_LAYER, 0,
                                reinterpret_cast<const uint8_t*>(&del),
                                sizeof(del));
    }

    void DeleteAll() {
        WireGraphicDelete del{};
        del.delete_type = 2;
        del.layer       = 0;
        parser_.SendInteraction(SUB_CMD_DELETE_LAYER, 0,
                                reinterpret_cast<const uint8_t*>(&del),
                                sizeof(del));
    }

private:
    RefereeParser& parser_;

    static constexpr uint8_t MAX_BATCH = 7;
    WireGraphicData buffer_[MAX_BATCH]{};
    uint8_t count_ = 0;

    static void SetName(WireGraphicData& g, const char name[3]) {
        g.figure_name[0] = static_cast<uint8_t>(name[0]);
        g.figure_name[1] = static_cast<uint8_t>(name[1]);
        g.figure_name[2] = static_cast<uint8_t>(name[2]);
    }
};

} // namespace referee
