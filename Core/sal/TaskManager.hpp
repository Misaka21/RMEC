#pragma once
#include "cmsis_os.h"
#include "sal_dwt.h"
#include "log.h"
#include "xTools.hpp"
#include <cstdint>
#include <functional>

/**
 * @brief 任务管理类,封装FreeRTOS任务创建与调试计时
 *
 * 非模板类,header-only实现。构造时自动创建FreeRTOS线程,
 * 线程入口函数先调用init_func一次,然后循环调用task_func + osDelay。
 * 使用两个DwtInstance测量任务周期和执行耗时。
 */
class TaskManager
{
public:
    struct TaskDbgInfo
    {
        float ex_dt     = 0;
        float ex_dt_mx  = 0;
        float ex_time   = 0;
        float ex_time_mx = 0;
    };

    struct TaskConfig
    {
        const char               *name       = "default";
        uint32_t                  stack_size  = 128;     // 单位: words
        osPriority                priority    = osPriorityNormal;
        uint32_t                  period_ms   = 1;
        std::function<void()>     init_func   = nullptr;
        std::function<void()>     task_func   = nullptr;
    };

private:
    std::function<void()> task_func_;
    std::function<void()> init_func_;
    uint32_t              period_ms_;
    osThreadDef_t         thread_def_;
    DwtInstance            dwt_dt_;     // 测量任务周期
    DwtInstance            dwt_time_;   // 测量任务执行耗时

    /**
     * @brief 静态跳板函数,转换arg为TaskManager*后调用成员函数
     *
     * @param arg 指向TaskManager实例的指针
     */
    static void TaskTrampoline(void const *arg)
    {
        TaskManager *self = (TaskManager *)arg;

        if (self->init_func_)
            self->init_func_();

        for (;;)
        {
            // 测量任务周期
            self->dbg_info.ex_dt = self->dwt_dt_.DwtGetDeltaT();
            if (self->dbg_info.ex_dt > self->dbg_info.ex_dt_mx)
                self->dbg_info.ex_dt_mx = self->dbg_info.ex_dt;

            // 测量任务执行耗时
            self->dwt_time_.DwtGetDeltaT(); // 重置计时起点
            self->task_func_();
            self->dbg_info.ex_time = self->dwt_time_.DwtGetDeltaT();
            if (self->dbg_info.ex_time > self->dbg_info.ex_time_mx)
                self->dbg_info.ex_time_mx = self->dbg_info.ex_time;

            osDelay(self->period_ms_);
        }
    }

public:
    osThreadId    task_handle = nullptr;
    TaskDbgInfo dbg_info;

    TaskManager() = default;

    /**
     * @brief 构造并创建FreeRTOS任务
     *
     * @param config 任务配置结构体
     */
    TaskManager(const TaskConfig &config)
        : task_func_(config.task_func),
          init_func_(config.init_func),
          period_ms_(config.period_ms)
    {
        if (task_func_ == nullptr)
            DEBUG_DEADLOCK("[TaskManager] task_func is nullptr");

        // 手动填充osThreadDef_t结构体
        thread_def_.name      = (char *)config.name;
        thread_def_.pthread    = TaskTrampoline;
        thread_def_.tpriority  = config.priority;
        thread_def_.instances  = 0;
        thread_def_.stacksize  = config.stack_size * 4; // osThreadDef_t.stacksize单位为bytes

        task_handle = osThreadCreate(&thread_def_, this);
        if (task_handle == nullptr)
            DEBUG_DEADLOCK("[TaskManager] osThreadCreate failed");
    }
};
