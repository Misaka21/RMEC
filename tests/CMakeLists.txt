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

# 注册一个测试目标: rmec_add_test(<name> <src>...)
# 被测 Core/ 源文件显式列出, 不 glob —— 哪些代码 host 可编译是显式契约
function(rmec_add_test name)
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
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    gtest_discover_tests(${name})
endfunction()

rmec_add_test(test_sanity unit/test_sanity.cpp)
rmec_add_test(test_crc unit/test_crc.cpp)
rmec_add_test(test_dt7_protocol
    unit/test_dt7_protocol.cpp
    ${RMEC_CORE}/module/remote/protocol/dt7_protocol.cpp
)
