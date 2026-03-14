#include "referee_task.hpp"
#include "referee.hpp"
#include "referee_protocol.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "daemon.hpp"

// ======================== 常规链路 (底盘板, 115200) ========================
// ISR 直接发布到 Topic, 无需独立 FreeRTOS 任务 (与 Remote 同模式)

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)

#include "sal_usart.h"
#include "usart.h"

static referee::RefereeParser* parser = nullptr;
static sal::UartInstance* ref_uart = nullptr;
static daemon::DaemonInstance* ref_daemon = nullptr;

void RefereeInit() {
    ref_daemon = new daemon::DaemonInstance({
        .timeout_ticks = 30,  // 300ms @ 100Hz daemon tick
        .on_offline = [](void*) { ref_uart->UartRestartRecv(); },
    });

    parser = new referee::RefereeParser(
        [](uint8_t* buf, uint16_t len) { ref_uart->UartSend(buf, len); },
        [](const referee::RefereeData& d) {
            referee_topic.Publish(d);
        }
    );

    sal::UartInstance::UartConfig uart_cfg{};
    uart_cfg.handle  = &REFEREE_UART_HANDLE;
    uart_cfg.rx_size = 256;
    uart_cfg.rx_type = sal::UartRxType::DMA_IDLE;
    uart_cfg.rx_cbk  = [](uint8_t* buf, uint16_t len) {
        parser->Parse(buf, len);
        ref_daemon->Reload();
    };
    uart_cfg.tx_type = sal::UartTxType::IT;
    ref_uart = new sal::UartInstance(uart_cfg);
    ref_uart->UartRestartRecv();
}

bool RefereeIsOnline() {
    return ref_daemon && ref_daemon->IsOnline();
}

referee::RefereeParser* GetRefereeParser() { return parser; }

#else // GIMBAL_BOARD — 常规链路不在云台板编译

void RefereeInit() {}
bool RefereeIsOnline() { return false; }
referee::RefereeParser* GetRefereeParser() { return nullptr; }

#endif

// ======================== 图传链路 (云台板, 921600) ========================
// ISR 解码, 当前图传链路不发布到 Topic (按需扩展)

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

#include "sal_usart.h"
#include "usart.h"

static referee::RefereeParser* vl_parser = nullptr;
static sal::UartInstance* vl_uart = nullptr;
static daemon::DaemonInstance* vl_daemon = nullptr;

void VideoLinkInit() {
    vl_daemon = new daemon::DaemonInstance({
        .timeout_ticks = 30,
        .on_offline = [](void*) { vl_uart->UartRestartRecv(); },
    });

    vl_parser = new referee::RefereeParser(
        [](uint8_t* buf, uint16_t len) { vl_uart->UartSend(buf, len); },
        nullptr  // 图传链路当前不发布到 Topic
    );

    sal::UartInstance::UartConfig uart_cfg{};
    uart_cfg.handle  = &VIDEO_LINK_UART_HANDLE;
    uart_cfg.rx_size = 256;
    uart_cfg.rx_type = sal::UartRxType::DMA_IDLE;
    uart_cfg.rx_cbk  = [](uint8_t* buf, uint16_t len) {
        vl_parser->Parse(buf, len);
        vl_daemon->Reload();
    };
    uart_cfg.tx_type = sal::UartTxType::IT;
    vl_uart = new sal::UartInstance(uart_cfg);
    vl_uart->UartRestartRecv();
}

#else // CHASSIS_BOARD — 图传链路不在底盘板编译

void VideoLinkInit() {}

#endif
