#include "referee_task.hpp"
#include "referee.hpp"
#include "referee_protocol.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "TaskManager.hpp"
#include "daemon.hpp"

// ======================== 常规链路 (底盘板, 115200) ========================

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)

#include "sal_usart.h"
#include "usart.h"

static referee::RefereeParser* parser = nullptr;
static sal::UartInstance* ref_uart = nullptr;
static daemon::DaemonInstance* ref_daemon = nullptr;

void RefereeTaskStart() {
    static TaskManager referee_task({
        .name       = "referee",
        .stack_size = 512,
        .priority   = osPriorityNormal,
        .period_ms  = 10,  // 100Hz

        .init_func = []() {
            ref_daemon = new daemon::DaemonInstance({
                .timeout_ticks = 30,  // 300ms @ 100Hz daemon tick
                .on_offline = [](void*) { ref_uart->UartRestartRecv(); },
            });

            parser = new referee::RefereeParser(
                [](uint8_t* buf, uint16_t len) { ref_uart->UartSend(buf, len); }
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
        },

        .task_func = []() {
            referee_topic.Publish(parser->Data());
        },
    });
}

bool RefereeIsOnline() {
    return ref_daemon && ref_daemon->IsOnline();
}

const referee::RefereeData& GetRefereeData() {
    return parser->Data();
}

#else // GIMBAL_BOARD — 常规链路不在云台板编译

void RefereeTaskStart() {}
bool RefereeIsOnline() { return false; }

static referee::RefereeData dummy_data{};
const referee::RefereeData& GetRefereeData() { return dummy_data; }

#endif

// ======================== 图传链路 (云台板, 921600) ========================

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

#include "sal_usart.h"
#include "usart.h"

static referee::RefereeParser* vl_parser = nullptr;
static sal::UartInstance* vl_uart = nullptr;
static daemon::DaemonInstance* vl_daemon = nullptr;

void VideoLinkTaskStart() {
    static TaskManager video_link_task({
        .name       = "vlink",
        .stack_size = 512,
        .priority   = osPriorityNormal,
        .period_ms  = 10,  // 100Hz

        .init_func = []() {
            vl_daemon = new daemon::DaemonInstance({
                .timeout_ticks = 30,
                .on_offline = [](void*) { vl_uart->UartRestartRecv(); },
            });

            vl_parser = new referee::RefereeParser(
                [](uint8_t* buf, uint16_t len) { vl_uart->UartSend(buf, len); }
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
        },

        .task_func = []() {
            // 图传链路主要用于自定义控制器数据
            // 如果需要, 可以在此发布到 Topic 或供 cmd_task 查询
        },
    });
}

#else // CHASSIS_BOARD — 图传链路不在底盘板编译

void VideoLinkTaskStart() {}

#endif
