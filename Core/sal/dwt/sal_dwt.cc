#include "sal_dwt.h"
#include "cmsis_os.h"
#include "xTools.hpp"

#include <chrono>

void DWTInstance::DWT_CNT_Update(void)
{
    static bit_locker bit_lock;
    static uint32_t CYCCNT_LAST; // 上锁资源,防止重入

    if (bit_lock.try_lock())
    {
        lock_guard guard(bit_lock);
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            cyc_round_cnt_++;
        CYCCNT_LAST = DWT->CYCCNT;
    }
    else
        return;
}

void DWTInstance::DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)cyc_round_cnt_ * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    dwt_time_.s = CNT_TEMP1;
    dwt_time_.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - dwt_time_.ms * CPU_FREQ_Hz_ms;
    dwt_time_.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

void DWTInstance::DWTInit(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;
    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    cyc_round_cnt_ = 0;

    DWT_CNT_Update();
}

float DWTInstance::DWTGetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = dwt_time_.s + dwt_time_.ms * 0.001f + dwt_time_.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWTInstance::DWTGetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = dwt_time_.s * 1000 + dwt_time_.ms + dwt_time_.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWTInstance::DWTGetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return dwt_time_.s * 1000000 + dwt_time_.ms * 1000 + dwt_time_.us;
}

void DWTInstance::DWTDelay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}

DWTInstance::DWTInstance()
{
}

float DWTInstance::DWTGetDeltaT()
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - dwt_cnt_)) / ((float)(CPU_FREQ_Hz));
    dwt_cnt_ = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWTInstance::DWTGetDeltaT64()
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - dwt_cnt_)) / ((double)(CPU_FREQ_Hz));
    dwt_cnt_ = cnt_now;

    DWT_CNT_Update();

    return dt;
}
