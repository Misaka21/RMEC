#pragma once

#include "main.h"
#include "log.h"
#include <stdint.h>

/**
 * @brief 该宏用于计算代码段执行时间,单位为ms,返回值为float类型
 *        计算得到的时间间隔同时还会通过RTT打印到日志终端,你也可以将你的dt变量添加到查看
 *
 * @attention 由于使用了lambda表达式,并在捕获列表中获取了全部的局部变量,因此可能会导致程序执行效率下降
 *            测试完毕后,请恢复原来的代码.也可以通过启用DISABLE_LOG_SYSTEM宏来关闭计时
 *            关闭后返回值为0.但此时仍有lambda表达式的开销(主要是捕获列表,而且开销实际上较大,但不影响计时的准确性)
 *
 * @param string 输出在log终端的提示字段 "[DWT] string = dt"
 * @param code   要执行的代码,可以传入多个语句,每行都需要以分号;结尾
 * @return float 代码段执行时间,单位为ms,若关闭日志系统则返回0
 */
#ifndef DISABLE_LOG_SYSTEM
#define TIME_ELAPSE(string, code)                             \
    [&]() -> float                                            \
    {                                                         \
        float tstart = DwtInstance::DwtGetTimeline_ms();      \
        code;                                                 \
        float dt = DwtInstance::DwtGetTimeline_ms() - tstart; \
        LOGINFO("[DWT] %s = %f s\r\n", string, dt);           \
        return dt;                                            \
    }
#else // 直接执行code,不打印日志,返回0
#define TIME_ELAPSE(string, code) \
    [&]() -> float                \
    {                             \
        code;                     \
        return 0;                 \
    }
#endif

/**
 * @brief 进入while执行code,直到超时
 *
 * @todo 找到更好的返回方案
 * @param code 要执行的代码片段
 * @param timeout 超时时间,单位为ms
 * @return true code执行完毕,没有超时就已经返回
 *         false 代码执行超时
 *
 * @attention 若希望中止,请在code中使用return, true表示执行成功, false表示失败
 *
 */
#define TIMEOUT(code, timeout)                                      \
    [&]() -> bool                                                   \
    {                                                               \
        float tstart = DwtInstance::DwtGetTimeline_ms();            \
        while (DwtInstance::DwtGetTimeline_ms() - tstart < timeout) \
        {                                                           \
            code;                                                   \
        }                                                           \
        return false;                                               \
    }

class DwtInstance
{

private:
    /* 内建时间类,维护时间轴供DWTGetTimeline()使用*/
    static struct DwtTime
    {
        uint32_t s;
        uint16_t ms;
        uint16_t us;
    } dwt_time_;

    // 初始化时获取
    static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;

    static uint32_t cyc_round_cnt_; // CYCCNT溢出次数,用于计算时间轴
    static uint64_t CYCCNT64;       // CYCCNT的总计数(判断溢出后会直接增加uint32_t_MAX)

    uint32_t dwt_cnt_; // 每个instance的计数器,用于提供间隔时间

    /**
     * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
     * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
     *
     * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
     *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
     *
     */
    static void DwtCntUpdate(void);

    /**
     * @brief DWT更新时间轴函数,会被三个timeline函数调用
     * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴
     *            否则CYCCNT溢出后定时和时间轴不准确
     */
    static void DwtSysTimeUpdate(void);

public:
    /**
     * @brief 初始化DWT,传入参数为CPU频率,单位MHz
     *
     * @param CPU_Freq_mHz c板为168MHz,A板为180MHz,和时钟树&时钟配置有关
     */
    static void DwtInit(uint32_t CPU_Freq_mHz);

public:
    /* ----------------------↓ 直接供外部调用 ↓----------------------*/

    // 当前时间轴,以初始化后的时间为基准
    static float DwtGetTimeline_s(void);
    static float DwtGetTimeline_ms(void);
    static uint64_t DwtGetTimeline_us(void);

    /**
     * @brief DWT延时函数,单位为秒/s
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒/s
     */
    static void DwtDelay(float Delay);

public:
    DwtInstance();
    ~DwtInstance() = default;
    /**
     * @brief 获取两次调用之间的时间间隔,单位为秒/s
     *
     * @param cnt_last 上一次调用的时间戳
     * @return float 时间间隔,单位为秒/s
     */
    float DwtGetDeltaT();

    /* DwtGetDeltaT()的双精度浮点版本 */
    double DwtGetDeltaT64();
};
