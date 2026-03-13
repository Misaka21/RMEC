#include "vofa_task.hpp"

#ifdef VOFA_ENABLED

#include "vofa.hpp"
#include "robot_def.hpp"
#include "TaskManager.hpp"
#include "usart.h"

inline constexpr uint8_t VOFA_CHANNELS = 8;

static vofa::Vofa<VOFA_CHANNELS>* plotter = nullptr;

void VofaTaskStart() {
    static TaskManager vofa_task({
        .name       = "vofa",
        .stack_size = 128,
        .priority   = osPriorityBelowNormal,
        .period_ms  = 10,

        .init_func = []() {
            plotter = new vofa::Vofa<VOFA_CHANNELS>({
                .uart_handle = &VOFA_UART_HANDLE,
            });
        },

        .task_func = []() {
            plotter->Send();
        },
    });
}

void VofaSetChannel(uint8_t ch, float val) {
    if (plotter) plotter->SetChannel(ch, val);
}

#endif
