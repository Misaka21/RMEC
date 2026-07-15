#pragma once
// 假 TaskManager: 不创建 FreeRTOS 线程, 只登记 init/task 到 test::TaskRegistry,
// 测试代码手动 RunTaskInit + StepTask 步进 (设计文档第 4 节, L3 不启动 FreeRTOS)
#include "cmsis_os.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace test {

struct TaskEntry {
    std::string name;
    uint32_t period_ms = 1;
    std::function<void()> init_func;
    std::function<void()> task_func;
};

std::vector<TaskEntry>& TaskRegistry();
void RunTaskInit(const char* name);
void StepTask(const char* name, uint32_t times = 1);

}  // namespace test

class TaskManager {
public:
    // 字段名与声明顺序必须与真 TaskManager::TaskConfig 一致 (app 层用指定初始化器)
    struct TaskConfig {
        const char*           name       = "default";
        uint32_t              stack_size = 128;
        osPriority            priority   = osPriorityNormal;
        uint32_t              period_ms  = 1;
        std::function<void()> init_func  = nullptr;
        std::function<void()> task_func  = nullptr;
    };

    TaskManager() = default;
    TaskManager(const TaskConfig& config);

    osThreadId task_handle = nullptr;
};
