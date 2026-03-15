#include <cstdio>
#include <cstdarg>
#include <cstring>

#include "log.h"
#include "sal_usart.h"

// // ==================== 旧版 RTT 实现 ====================
// #include "SEGGER_RTT.h"
//
// void BSPLogInit()
// {
//     SEGGER_RTT_Init();
// }
//
// int PrintLog(const char *fmt, ...)
// {
//     va_list args;
//     va_start(args, fmt);
//     int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args);
//     va_end(args);
//     return n;
// }

// ==================== UART 实现 ====================

static sal::UartInstance* log_uart = nullptr;
static char log_buf[256];

void BSPLogInit(UART_HandleTypeDef* handle)
{
    static sal::UartInstance::UartConfig cfg{};
    cfg.handle  = handle;
    cfg.tx_type = sal::UartTxType::BLOCK;
    log_uart = new sal::UartInstance(cfg);
}

int PrintLog(const char *fmt, ...)
{
    if (!log_uart) return 0;

    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(log_buf, sizeof(log_buf), fmt, args);
    va_end(args);

    if (n > 0) {
        log_uart->UartSend(reinterpret_cast<uint8_t*>(log_buf),
                           n < (int)sizeof(log_buf) ? n : (int)sizeof(log_buf) - 1);
    }
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
