#pragma once
// 假 cmsis_os: 仅提供类型与常量, 线程创建被假 TaskManager 接管, 不会真正调用

typedef enum {
    osPriorityIdle        = -3,
    osPriorityLow         = -2,
    osPriorityBelowNormal = -1,
    osPriorityNormal      = 0,
    osPriorityAboveNormal = 1,
    osPriorityHigh        = 2,
    osPriorityRealtime    = 3,
} osPriority;

typedef void* osThreadId;
