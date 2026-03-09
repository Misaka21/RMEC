#include "sal_flash.h"
#include "log.h"

namespace sal {

    /* ========================= 扇区查找表 =========================
     * 用于将Flash地址映射到HAL扇区编号,替代冗长的if-else链
     * 从高地址向低地址遍历,第一个 addr <= target 的条目即为目标扇区
     */
    struct SectorInfo {
        uint32_t addr;       // 扇区起始地址
        uint32_t sector_id;  // HAL定义的扇区编号 (FLASH_SECTOR_x)
    };

    static constexpr SectorInfo sector_table[] = {
        {Flash::SECTOR_0_ADDR,  FLASH_SECTOR_0},
        {Flash::SECTOR_1_ADDR,  FLASH_SECTOR_1},
        {Flash::SECTOR_2_ADDR,  FLASH_SECTOR_2},
        {Flash::SECTOR_3_ADDR,  FLASH_SECTOR_3},
        {Flash::SECTOR_4_ADDR,  FLASH_SECTOR_4},
        {Flash::SECTOR_5_ADDR,  FLASH_SECTOR_5},
        {Flash::SECTOR_6_ADDR,  FLASH_SECTOR_6},
        {Flash::SECTOR_7_ADDR,  FLASH_SECTOR_7},
        {Flash::SECTOR_8_ADDR,  FLASH_SECTOR_8},
        {Flash::SECTOR_9_ADDR,  FLASH_SECTOR_9},
        {Flash::SECTOR_10_ADDR, FLASH_SECTOR_10},
        {Flash::SECTOR_11_ADDR, FLASH_SECTOR_11},
    };

    static constexpr uint32_t SECTOR_COUNT = sizeof(sector_table) / sizeof(sector_table[0]);

    /**
     * @brief 根据Flash地址查找所在扇区编号
     *        从表尾(高地址)向表头遍历,找到第一个起始地址 <= address 的扇区
     *
     * @param address Flash地址
     * @return uint32_t HAL扇区编号
     */
    uint32_t Flash::GetSector(uint32_t address)
    {
        // 从最后一个扇区开始向前查找,找到起始地址不大于address的扇区
        for (uint32_t i = SECTOR_COUNT - 1; i > 0; --i) {
            if (address >= sector_table[i].addr)
                return sector_table[i].sector_id;
        }
        // 地址落在Sector 0范围内(或非法的低地址)
        return sector_table[0].sector_id;
    }

    /**
     * @brief 获取下一个扇区的起始地址
     *        正向遍历查找表,找到包含address的扇区后返回下一条目的地址
     *
     * @param address Flash地址
     * @return uint32_t 下一个扇区起始地址,若在最后一个扇区则返回FLASH_END_ADDR
     */
    uint32_t Flash::GetNextSectorAddr(uint32_t address)
    {
        for (uint32_t i = 0; i < SECTOR_COUNT - 1; ++i) {
            if (address >= sector_table[i].addr && address < sector_table[i + 1].addr)
                return sector_table[i + 1].addr;
        }
        // 地址已在最后一个扇区(Sector 11),返回Flash结束地址
        return FLASH_END_ADDR;
    }

    /**
     * @brief 擦除Flash扇区
     *
     * @note 流程: 解锁Flash → 执行扇区擦除 → 重新加锁
     *       VoltageRange选择RANGE_3(2.7~3.6V),与C板供电匹配
     */
    HAL_StatusTypeDef Flash::Erase(uint32_t address, uint16_t num_sectors)
    {
        FLASH_EraseInitTypeDef erase_init;
        uint32_t sector_error = 0; // HAL会在此返回出错的扇区号

        erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;   // 按扇区擦除
        erase_init.Sector       = GetSector(address);        // 自动定位起始扇区
        erase_init.NbSectors    = num_sectors;               // 连续擦除扇区数
        erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;     // 2.7~3.6V,支持32位编程

        HAL_FLASH_Unlock();  // 解锁Flash控制寄存器
        HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
        HAL_FLASH_Lock();    // 重新加锁,防止误操作

        if (status != HAL_OK)
            LOGERROR("[sal::Flash] erase failed at sector %d, error=0x%x", erase_init.Sector, sector_error);

        return status;
    }

    /**
     * @brief 以字(32bit)为单位逐个写入Flash
     *
     * @note 流程: 解锁Flash → 循环写入每个word → 重新加锁
     *       FLASH_TYPEPROGRAM_WORD表示每次编程32位(4字节)
     *       写入失败时会立即加锁并返回false,避免Flash处于解锁状态
     */
    bool Flash::Write(uint32_t address, const uint32_t *data, uint32_t len)
    {
        HAL_FLASH_Unlock();

        for (uint32_t i = 0; i < len; ++i) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]) != HAL_OK) {
                LOGERROR("[sal::Flash] write failed at 0x%08x", address);
                HAL_FLASH_Lock(); // 写入失败也要加锁
                return false;
            }
            address += 4; // 每次写入4字节,地址递增
        }

        HAL_FLASH_Lock();
        return true;
    }

    /**
     * @brief 从Flash读取数据(本质为memcpy)
     *
     * @note STM32的Flash地址空间是内存映射的,可以直接通过指针读取,
     *       不需要解锁Flash,也不需要特殊的读取流程
     *       len的单位是uint32_t(4字节),实际读取字节数 = len * 4
     */
    void Flash::Read(uint32_t address, uint32_t *buf, uint32_t len)
    {
        memcpy(buf, reinterpret_cast<const void *>(address), len * 4);
    }

} // namespace sal
