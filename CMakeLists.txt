cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 指定工具链
set(CMAKE_C_COMPILER_FORCED TRUE) # skip compiler test
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size) 
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
# 指定工程名称和语言类型
project(pwf_fw C CXX ASM) 

# 选择构建类型
set(CMAKE_BUILD_TYPE Debug) # Debug Release RelWithDebInfo MinSizeRel

# board specific settings, arch/fpu/instruction
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16)
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ToolChain/STM32F407IGHX_FLASH.ld") # 指定链接脚本
set(CMSISCORE "${CMAKE_SOURCE_DIR}/Drivers/CMSIS")
set(DSP_NAME "libCMSISDSP.a") # 指定DSP库名称

# 构建选项
option(BUILD_DSP "Build CMSIS-DSP library from source" ON)

if(BUILD_DSP)
    add_subdirectory(${CMAKE_SOURCE_DIR}/ExtLibs/DSP/Source)
endif()

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -Werror -fmessage-length=0 # basic options
                    -ffunction-sections -fdata-sections -fno-common # optimize options 
                    )
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
                 -lc -lstdc++ -lm -lnosys # lib options
                 -Wl,--gc-sections -flto -specs=nano.specs -specs=nosys.specs # optimize options
                 -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
                 ) # if your executable is too large , try option '-s' to strip symbols

# add_compile_definitions() works for compile stage
# while add_definitions() works for both compile and link stage
add_definitions(
    -DUSE_HAL_DRIVER 
    -DSTM32F407xx 
    -DARM_MATH_CM4 
    -DARM_MATH_MATRIX_CHECK 
    -DARM_MATH_ROUNDING 
    -DARM_MATH_LOOPUNROLL 
    -DDISABLEFLOAT16
    ) # need -D<macro> to define macro

# add inc
include(${CMAKE_SOURCE_DIR}/ToolChain/find_header.cmake)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/ExtLibs/MatrixRobotics)
include_directories(${CMAKE_SOURCE_DIR}/ExtLibs/DSP/Include)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Core)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Drivers)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Middlewares)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Inc)

# add source, only surfix .c .cpp
file(GLOB_RECURSE SOURCES
    "Drivers/*.c"
    "Src/*.c"
    "Middlewares/*.c"
    "Core/*.cpp"
    "Core/*.cc"
    "Core/*.hpp"
    "ExtLibs/MatrixRobotics/*.cpp"
)

# 汇编文件路径
set(ASM_SOURCES
    Toolchain/startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    )
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG) # ESC Debug
endif ()

# build binary and hex file
add_executable(${PROJECT_NAME}.elf  ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})

if(BUILD_DSP)
    add_dependencies(${PROJECT_NAME}.elf CMSISDSP) # build DSP lib
    target_link_libraries(${PROJECT_NAME}.elf CMSISDSP) # link DSP lib
endif()

add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin file..."
    COMMENT "EXCUTABLE SIZE:"
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
  )