#include "vision_task.hpp"
#include "vision_comm.hpp"
#include "vision_protocol.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "general_def.hpp"
#include "TaskManager.hpp"
#include "daemon.hpp"

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

#ifdef VISION_USE_VCP
#include "sal_usb.h"
static sal::UsbInstance* usb = nullptr;
#else
#include "sal_usart.h"
#include "usart.h"
static sal::UartInstance* uart = nullptr;
#endif

static vision::VisionComm* vis = nullptr;
static daemon::DaemonInstance* vis_daemon = nullptr;
static TopicReader<InsData>* ins_reader = nullptr;

// 发送参数 (由 cmd_task 写, vision_task 读, 单字节/float 原子性足够)
static vision::VisionTxData tx_state{};

void VisionTaskStart() {
    ins_reader = ins_topic.Subscribe();

    static TaskManager vision_task({
        .name       = "vision",
        .stack_size = 256,
        .priority   = osPriorityAboveNormal,
        .period_ms  = 1,

        .init_func = []() {
#ifdef VISION_USE_VCP
            // ---- USB VCP 路径 ----
            vis_daemon = new daemon::DaemonInstance({.timeout_ticks = 50});

            vis = new vision::VisionComm(
                [](uint8_t* buf, uint16_t len) { usb->Transmit(buf, len); },
                [](const vision::VisionRxData& d) {
                    vision_topic.Publish(d);
                    vis_daemon->Reload();
                });

            sal::UsbInstance::UsbConfig usb_cfg{};
            usb_cfg.rx_cbk = [](uint8_t* buf, uint16_t len) {
                vis->OnReceive(buf, len);
            };
            usb = new sal::UsbInstance(usb_cfg);
#else
            // ---- UART 路径 ----
            vis_daemon = new daemon::DaemonInstance({
                .timeout_ticks = 50,
                .on_offline = [](void*) { uart->UartRestartRecv(); },
            });

            vis = new vision::VisionComm(
                [](uint8_t* buf, uint16_t len) { uart->UartSend(buf, len); },
                [](const vision::VisionRxData& d) {
                    vision_topic.Publish(d);
                    vis_daemon->Reload();
                });

            sal::UartInstance::UartConfig uart_cfg{};
            uart_cfg.handle  = &VISION_UART_HANDLE;
            uart_cfg.rx_size = vision::FRAME_SIZE;
            uart_cfg.rx_type = sal::UartRxType::DMA_IDLE;
            uart_cfg.rx_cbk  = [](uint8_t* buf, uint16_t len) {
                vis->OnReceive(buf, len);
            };
            uart_cfg.tx_type = sal::UartTxType::IT;
            uart = new sal::UartInstance(uart_cfg);
            uart->UartRestartRecv();
#endif
        },

        .task_func = []() {
            InsData ins{};
            if (ins_reader->Read(ins)) {
                tx_state.yaw   = ins.euler[0] * DEGREE_2_RAD;
                tx_state.pitch = ins.euler[1] * DEGREE_2_RAD;
                tx_state.roll  = ins.euler[2] * DEGREE_2_RAD;
            }

            vis->Send(tx_state);
        },
    });
}

void VisionSetMode(vision::AimMode mode, vision::EnemyColor color,
                   float bullet_speed) {
    tx_state.aim_mode     = mode;
    tx_state.enemy_color  = color;
    tx_state.bullet_speed = bullet_speed;
}

void VisionSetAimingLock(bool lock) {
    tx_state.aiming_lock = lock ? 1 : 0;
}

bool VisionIsOnline() {
    return vis_daemon && vis_daemon->IsOnline();
}

#else // CHASSIS_BOARD

void VisionTaskStart() {}
void VisionSetMode(vision::AimMode, vision::EnemyColor, float) {}
void VisionSetAimingLock(bool) {}
bool VisionIsOnline() { return false; }

#endif
