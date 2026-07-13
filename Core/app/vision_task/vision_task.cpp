#include "vision_task.hpp"
#include "vision_comm.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "math.hpp"
#include "TaskManager.hpp"
#include "daemon.hpp"

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

static vision::VisionComm* vis = nullptr;
static daemon::DaemonInstance* vis_daemon = nullptr;
static TopicReader<InsData>* ins_reader = nullptr;

// 发送参数 (由 robot_task 写, vision_task 读, 单字节/float 原子性足够)
static vision::VisionTxData tx_state{};

void VisionTaskStart() {
    ins_reader = ins_topic.Subscribe();

    static TaskManager vision_task({
        .name       = "vision",
        .stack_size = 256,
        .priority   = osPriorityAboveNormal,
        .period_ms  = 1,

        .init_func = []() {
            vis_daemon = new daemon::DaemonInstance({
                .timeout_ticks = 50,
                .on_offline = [](void*) { if (vis) vis->RestartRx(); },
            });

            vision::VisionConfig cfg{};
#ifdef VISION_USE_VCP
            cfg.transport = vision::VisionTransport::USB_VCP;
#else
            cfg.transport = vision::VisionTransport::UART;
            cfg.uart_handle = &VISION_UART_HANDLE;
#endif

            vis = new vision::VisionComm(
                cfg,
                [](const vision::VisionRxData& d) {
                    vision_topic.Publish(d);
                    vis_daemon->Reload();
                });
        },

        .task_func = []() {
            InsData ins{};
            if (ins_reader->Read(ins)) {
                tx_state.yaw   = ins.euler[0] * math::DEGREE_2_RAD;
                tx_state.pitch = ins.euler[1] * math::DEGREE_2_RAD;
                tx_state.roll  = ins.euler[2] * math::DEGREE_2_RAD;
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
