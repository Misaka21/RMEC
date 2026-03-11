#include "remote_task.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"
#include "daemon.hpp"

#include "usart.h"

static Dt7Remote* rc = nullptr;
static daemon::DaemonInstance* rc_daemon = nullptr;

void RemoteInit() {
    remote::RemoteConfig cfg{};
    cfg.uart_handle = &RC_UART_HANDLE;

    // 1. 创建 daemon 实例: 10 tick @ 100Hz = 100ms 超时
    rc_daemon = new daemon::DaemonInstance({
        .reload_count = 10,
        .callback = []() {
            rc->Reset();
            rc->RestartRx();
        },
    });

    // 2. 创建遥控器: ISR 回调发布 + 喂狗 (唯一 Topic 写者)
    rc = new Dt7Remote(cfg, [](const remote::Dt7Data& d) {
        remote_topic.Publish(d);
        rc_daemon->Reload();
    });
}

bool RemoteIsOnline() {
    return rc_daemon && rc_daemon->IsOnline();
}

Dt7Remote* GetRemote() { return rc; }
