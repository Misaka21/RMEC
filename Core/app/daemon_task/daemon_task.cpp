#include "daemon_task.hpp"
#include "daemon.hpp"
#include "TaskManager.hpp"

void DaemonTaskStart() {
    static TaskManager daemon_task({
        .name       = "daemon",
        .stack_size = 128,
        .priority   = osPriorityNormal,
        .period_ms  = 10,

        .task_func = []() {
            daemon::DaemonInstance::TickAll();
        },
    });
}
