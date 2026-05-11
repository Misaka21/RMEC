#include "referee_task.hpp"
#include "referee.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "daemon.hpp"

// ======================== 常规链路 (底盘板, 115200) ========================
// ISR 直接发布到 Topic, 无需独立 FreeRTOS 任务 (与 Remote 同模式)

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)

static referee::Referee* ref = nullptr;
static daemon::DaemonInstance* ref_daemon = nullptr;

void RefereeInit() {
    referee::RefereeConfig cfg{};
    cfg.uart_handle = &REFEREE_UART_HANDLE;

    ref_daemon = new daemon::DaemonInstance({
        .timeout_ticks = 30,  // 300ms @ 100Hz daemon tick
        .on_offline = [](void*) { if (ref) ref->RestartRx(); },
    });

    ref = new referee::Referee(
        cfg,
        [](const referee::RefereeData& d) {
            referee_topic.Publish(d);
            ref_daemon->Reload();
        }
    );
}

bool RefereeIsOnline() {
    return ref_daemon && ref_daemon->IsOnline();
}

referee::RefereeParser* GetRefereeParser() {
    return ref ? ref->Parser() : nullptr;
}

#else // GIMBAL_BOARD — 常规链路不在云台板编译

void RefereeInit() {}
bool RefereeIsOnline() { return false; }
referee::RefereeParser* GetRefereeParser() { return nullptr; }

#endif

// ======================== 图传链路 (云台板, 921600) ========================
// ISR 解码, 当前图传链路不发布到 Topic (按需扩展)

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

static referee::Referee* vl_ref = nullptr;
static daemon::DaemonInstance* vl_daemon = nullptr;

void VideoLinkInit() {
    referee::RefereeConfig cfg{};
    cfg.uart_handle = &VIDEO_LINK_UART_HANDLE;

    vl_daemon = new daemon::DaemonInstance({
        .timeout_ticks = 30,
        .on_offline = [](void*) { if (vl_ref) vl_ref->RestartRx(); },
    });

    vl_ref = new referee::Referee(
        cfg,
        [](const referee::RefereeData&) {
            vl_daemon->Reload();
        }
    );
}

#else // CHASSIS_BOARD — 图传链路不在底盘板编译

void VideoLinkInit() {}

#endif
