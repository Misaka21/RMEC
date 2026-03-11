#include "remote_task.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"

#include "usart.h"

static Dt7Remote* rc = nullptr;
static uint32_t   last_seq = 0;
static uint16_t   offline_cnt = 0;
static uint16_t   offline_threshold = 100;
static bool       offline_published = true;

void RemoteInit() {
    remote::RemoteConfig cfg{};
    cfg.uart_handle = &RC_UART_HANDLE;
    cfg.offline_threshold = 100;

    rc = new Dt7Remote(cfg, [](const remote::Dt7Data& d) {
        remote_topic.Publish(d);
    });
    last_seq = 0;
    offline_cnt = cfg.offline_threshold;
    offline_threshold = cfg.offline_threshold;
    offline_published = true;
}

void RemoteOfflineCheck() {
    if (!rc) return;

    uint32_t seq = rc->SnapshotSeq();
    if (seq != last_seq) {
        last_seq = seq;
        offline_cnt = 0;
        offline_published = false;
        return;
    }

    if (offline_cnt < offline_threshold) {
        offline_cnt++;
    }

    if (offline_cnt >= offline_threshold && !offline_published) {
        rc->ResetAndPublish();
        offline_published = true;
    }
}

Dt7Remote* GetRemote() { return rc; }
