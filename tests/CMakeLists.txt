cmake_minimum_required(VERSION 3.16)
project(rmec_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(RMEC_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../Core)

enable_testing()
include(GoogleTest)

# fake 支撑源: 假寄存器/句柄定义 + 链接期替换的假 sal 实现
set(RMEC_FAKE_SUPPORT
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_registers.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_sal_usart.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_sal_can.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal/fake_task_manager.cc
)

# 内部: 建测试可执行目标
# 被测 Core/ 源文件显式列出, 不 glob —— 哪些代码 host 可编译是显式契约
function(_rmec_test_target name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE GTest::gtest_main)
    target_include_directories(${name} PRIVATE
        ${RMEC_CORE}/module
        ${RMEC_CORE}/module/referee
        ${RMEC_CORE}/module/vision
        ${RMEC_CORE}/module/remote/protocol
        ${RMEC_CORE}/module/motor
        ${RMEC_CORE}/module/motor/driver
        ${RMEC_CORE}/module/algorithm
        ${RMEC_CORE}/sal
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
endfunction()

# 内部: fake_hal/fake_sal 编译环境
# fake_hal 目录以 BEFORE 优先, 让 Core/ 源里的 "usart.h"/"can.h"/"main.h"/
# "cmsis_os.h"/"TaskManager.hpp" 解析到假头; sal_*.h 用真头, 实现链接 fake_sal
function(_rmec_fake_env name)
    target_include_directories(${name} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_hal
        ${CMAKE_CURRENT_SOURCE_DIR}/support/fake_sal
    )
    target_include_directories(${name} PRIVATE
        ${RMEC_CORE}/sal/usart
        ${RMEC_CORE}/sal/can
        ${RMEC_CORE}/sal/dwt
        ${RMEC_CORE}/sal/log
        ${RMEC_CORE}/module/motor/controller
        ${RMEC_CORE}/module/daemon
        ${RMEC_CORE}/module/imu
        ${RMEC_CORE}/app
        ${RMEC_CORE}/app/robot_task
        ${RMEC_CORE}/app/motor_task
        ${RMEC_CORE}/app/remote_task
        ${RMEC_CORE}/app/vision_task
        ${RMEC_CORE}/app/referee_task
        ${RMEC_CORE}/app/ins_task
    )
endfunction()

# 注册纯逻辑单测 (无 HAL 依赖): 逐用例发现, 各用例独立进程
function(rmec_add_test name)
    _rmec_test_target(${name} ${ARGN})
    gtest_discover_tests(${name})
endfunction()

# 注册 fake 环境单测: 用例相互独立, 逐用例发现
function(rmec_add_fake_test name)
    _rmec_test_target(${name} ${ARGN} ${RMEC_FAKE_SUPPORT})
    _rmec_fake_env(${name})
    gtest_discover_tests(${name})
endfunction()

# 注册 L3 flow 测试: 用例共享任务静态状态并按声明顺序构成时间线,
# 整个二进制注册为单个 ctest 用例, 保证同进程顺序执行 (设计 6.2 约束)
function(rmec_add_flow_test name)
    _rmec_test_target(${name} ${ARGN} ${RMEC_FAKE_SUPPORT})
    _rmec_fake_env(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

rmec_add_test(test_sanity unit/test_sanity.cpp)
rmec_add_test(test_crc unit/test_crc.cpp)
rmec_add_test(test_dt7_protocol
    unit/test_dt7_protocol.cpp
    ${RMEC_CORE}/module/remote/protocol/dt7_protocol.cpp
)
rmec_add_test(test_mit_codec unit/test_mit_codec.cpp)
rmec_add_test(test_power_limiter
    unit/test_power_limiter.cpp
    ${RMEC_CORE}/module/motor/power_limiter.cpp
)
rmec_add_test(test_loop_queue unit/test_loop_queue.cpp)
rmec_add_test(test_pid
    unit/test_pid.cpp
    ${RMEC_CORE}/module/algorithm/pid_controller.cpp
)
rmec_add_fake_test(test_float2str
    unit/test_float2str.cpp
    ${RMEC_CORE}/sal/log/log.cc
)
rmec_add_fake_test(test_dwt_math
    unit/test_dwt_math.cpp
    ${RMEC_CORE}/sal/dwt/sal_dwt.cc
)
rmec_add_fake_test(test_referee_parser
    unit/test_referee_parser.cpp
    ${RMEC_CORE}/module/referee/referee.cpp
)
rmec_add_flow_test(test_shoot_loader
    flow/test_shoot_loader.cpp
    ${RMEC_CORE}/app/motor_task/shoot_motors.cpp
    ${RMEC_CORE}/module/motor/driver/dji_driver.cpp
    ${RMEC_CORE}/module/algorithm/pid_controller.cpp
    ${RMEC_CORE}/sal/log/log.cc
)
