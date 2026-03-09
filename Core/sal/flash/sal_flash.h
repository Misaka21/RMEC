#pragma once

// HAL
#include "main.h"
// STL
#include <cstdint>
#include <cstring>

/**
 * @brief Flash模块提供对STM32F407IGH内部Flash的读/写/擦除操作
 *
 * @note  STM32F407IGH为单Bank,共1MB Flash,分为12个扇区(Sector 0~11)
 *        扇区大小不均匀: Sector 0~3为16KB, Sector 4为64KB, Sector 5~11为128KB
 *
 * @attention 1. Flash写入前必须先擦除,擦除的最小单位是一个完整的扇区
 *            2. 擦除/写入操作期间HAL会短暂关闭中断,避免在高实时性任务中调用
 *            3. Sector 0~4 通常存放程序代码,建议使用 Sector 5~11 存储用户参数
 *            4. Flash有擦写寿命限制(约10000次),不要在循环中频繁写入
 *
 * @example 典型用途: 掉电保存PID参数、校准偏移量等
 *          sal::Flash::Erase(sal::Flash::SECTOR_11_ADDR, 1);
 *          sal::Flash::Write(sal::Flash::SECTOR_11_ADDR, data, len);
 *          sal::Flash::Read(sal::Flash::SECTOR_11_ADDR, buf, len);
 */
namespace sal {

    /**
     * @brief 内部Flash操作的纯静态工具类,不需要实例化
     *
     * @note 构造函数被delete,所有方法均为static,作为命名空间内的工具集使用
     */
    class Flash {
    public:
        /* ========================= 扇区地址定义 =========================
         * STM32F407IGH: 单Bank, 1MB Flash, Sector 0~11
         * 地址范围: 0x0800_0000 ~ 0x080F_FFFF
         */
        static constexpr uint32_t SECTOR_0_ADDR  = 0x08000000; // 16KB  (程序代码区)
        static constexpr uint32_t SECTOR_1_ADDR  = 0x08004000; // 16KB  (程序代码区)
        static constexpr uint32_t SECTOR_2_ADDR  = 0x08008000; // 16KB  (程序代码区)
        static constexpr uint32_t SECTOR_3_ADDR  = 0x0800C000; // 16KB  (程序代码区)
        static constexpr uint32_t SECTOR_4_ADDR  = 0x08010000; // 64KB  (程序代码区)
        static constexpr uint32_t SECTOR_5_ADDR  = 0x08020000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_6_ADDR  = 0x08040000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_7_ADDR  = 0x08060000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_8_ADDR  = 0x08080000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_9_ADDR  = 0x080A0000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_10_ADDR = 0x080C0000; // 128KB (可用于参数存储)
        static constexpr uint32_t SECTOR_11_ADDR = 0x080E0000; // 128KB (可用于参数存储)
        static constexpr uint32_t FLASH_END_ADDR = 0x08100000; // Flash结束地址(不可访问)

        // 禁止实例化,所有方法均为static
        Flash() = delete;

        /**
         * @brief 擦除Flash扇区
         *
         * @param address 起始地址,会自动定位到该地址所属的扇区
         * @param num_sectors 从该扇区开始连续擦除的扇区数量
         * @return HAL_OK 擦除成功, 其他值表示失败
         *
         * @attention 擦除以扇区为单位,即使只需要修改1字节也必须擦除整个扇区
         *            擦除前请确保该扇区的有用数据已经备份
         */
        static HAL_StatusTypeDef Erase(uint32_t address, uint16_t num_sectors);

        /**
         * @brief 以字(32bit/4字节)为单位向Flash写入数据
         *
         * @param address 写入起始地址,须4字节对齐
         * @param data 写入数据的指针,以uint32_t为单位
         * @param len 数据长度,以uint32_t计(实际写入字节数 = len * 4)
         * @return true 写入成功
         * @return false 写入失败(会通过LOGERROR输出失败地址)
         *
         * @attention 写入前必须先调用Erase()擦除目标区域,向未擦除的区域写入会失败
         */
        static bool Write(uint32_t address, const uint32_t *data, uint32_t len);

        /**
         * @brief 从Flash读取数据到缓冲区
         *
         * @param address 读取起始地址
         * @param buf 目标缓冲区指针
         * @param len 数据长度,以uint32_t计(实际读取字节数 = len * 4)
         *
         * @note Flash地址空间可以直接通过指针访问(内存映射),
         *       此函数本质是memcpy,不需要解锁Flash
         */
        static void Read(uint32_t address, uint32_t *buf, uint32_t len);

        /**
         * @brief 根据Flash地址获取其所在的扇区编号
         *
         * @param address Flash地址
         * @return uint32_t HAL定义的扇区编号(FLASH_SECTOR_0 ~ FLASH_SECTOR_11)
         *
         * @note 返回值可直接用于FLASH_EraseInitTypeDef.Sector字段
         */
        static uint32_t GetSector(uint32_t address);

        /**
         * @brief 获取给定地址所在扇区的下一个扇区的起始地址
         *
         * @param address Flash地址
         * @return uint32_t 下一个扇区的起始地址; 若已是最后一个扇区则返回FLASH_END_ADDR
         *
         * @note 用于判断写入是否会越过当前扇区边界
         */
        static uint32_t GetNextSectorAddr(uint32_t address);
    };

} // namespace sal
