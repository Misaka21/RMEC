#include "remote_task.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"

#include "usart.h"

static Dt7Remote* rc = nullptr;

void RemoteInit() {
    remote::RemoteConfig cfg{};
    cfg.uart_handle = &RC_UART_HANDLE;

    rc = new Dt7Remote(cfg, [](const remote::Dt7Data& d) {
        remote_topic.Publish(d);
    });
}

Dt7Remote* GetRemote() { return rc; }
