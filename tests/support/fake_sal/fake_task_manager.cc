#include "TaskManager.hpp"

namespace test {

std::vector<TaskEntry>& TaskRegistry() {
    static std::vector<TaskEntry> r;
    return r;
}

void RunTaskInit(const char* name) {
    for (auto& t : TaskRegistry())
        if (t.name == name && t.init_func) t.init_func();
}

void StepTask(const char* name, uint32_t times) {
    for (auto& t : TaskRegistry())
        if (t.name == name && t.task_func)
            for (uint32_t i = 0; i < times; ++i) t.task_func();
}

}  // namespace test

TaskManager::TaskManager(const TaskConfig& config) {
    test::TaskRegistry().push_back(
        {config.name, config.period_ms, config.init_func, config.task_func});
    task_handle = this;
}
