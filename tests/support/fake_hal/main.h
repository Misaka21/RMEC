#pragma once
// 假 CubeMX main.h: DWT/CoreDebug 寄存器打桩, 供 sal_dwt 在宿主机编译与测试注入
#include <stdint.h>

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} FakeDwtType;

typedef struct {
    volatile uint32_t DEMCR;
} FakeCoreDebugType;

#ifdef __cplusplus
extern "C" {
#endif

extern FakeDwtType       fake_dwt;
extern FakeCoreDebugType fake_core_debug;

#ifdef __cplusplus
}
#endif

#define DWT (&fake_dwt)
#define CoreDebug (&fake_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0)
