#include "sal_i2c.h"
#include "log.h"

namespace sal {

    std::vector<I2CInstance::I2CPtr> I2CInstance::instance_list_;

    /**
     * @brief I2C实例构造函数
     *
     * @param config I2C配置结构体
     */
    I2CInstance::I2CInstance(const I2CConfig &config)
        : handle_(config.handle),
          dev_addr_(config.dev_addr << 1), // HAL需要左移1位的地址
          xfer_type_(config.xfer_type),
          tx_cbk_(config.tx_cbk),
          rx_cbk_(config.rx_cbk)
    {
        if (handle_ == nullptr)
            DEBUG_DEADLOCK("[sal::I2C] handle is nullptr");

        // I2C3未配置DMA stream,不支持DMA模式
        if (handle_->Instance == I2C3 && xfer_type_ == I2C_XFER_DMA)
            DEBUG_DEADLOCK("[sal::I2C] I2C3 does not support DMA");

        instance_list_.emplace_back(std::shared_ptr<I2CInstance>(this));
    }

    /**
     * @brief I2C寄存器读取
     *
     * @param reg 寄存器地址
     * @param data 接收缓冲区
     * @param size 读取字节数
     * @param timeout 超时时间(仅阻塞模式有效)
     */
    HAL_StatusTypeDef I2CInstance::MemRead(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2C_XFER_BLOCK:
            return HAL_I2C_Mem_Read(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size, timeout);
        case I2C_XFER_IT:
            return HAL_I2C_Mem_Read_IT(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        case I2C_XFER_DMA:
            return HAL_I2C_Mem_Read_DMA(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief I2C寄存器写入
     *
     * @param reg 寄存器地址
     * @param data 发送数据
     * @param size 写入字节数
     * @param timeout 超时时间(仅阻塞模式有效)
     */
    HAL_StatusTypeDef I2CInstance::MemWrite(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2C_XFER_BLOCK:
            return HAL_I2C_Mem_Write(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size, timeout);
        case I2C_XFER_IT:
            return HAL_I2C_Mem_Write_IT(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        case I2C_XFER_DMA:
            return HAL_I2C_Mem_Write_DMA(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief I2C原始发送
     *
     * @param data 发送数据
     * @param size 发送字节数
     * @param timeout 超时时间(仅阻塞模式有效)
     */
    HAL_StatusTypeDef I2CInstance::Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2C_XFER_BLOCK:
            return HAL_I2C_Master_Transmit(handle_, dev_addr_, data, size, timeout);
        case I2C_XFER_IT:
            return HAL_I2C_Master_Transmit_IT(handle_, dev_addr_, data, size);
        case I2C_XFER_DMA:
            return HAL_I2C_Master_Transmit_DMA(handle_, dev_addr_, data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief I2C原始接收
     *
     * @param data 接收缓冲区
     * @param size 接收字节数
     * @param timeout 超时时间(仅阻塞模式有效)
     */
    HAL_StatusTypeDef I2CInstance::Receive(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2C_XFER_BLOCK:
            return HAL_I2C_Master_Receive(handle_, dev_addr_, data, size, timeout);
        case I2C_XFER_IT:
            return HAL_I2C_Master_Receive_IT(handle_, dev_addr_, data, size);
        case I2C_XFER_DMA:
            return HAL_I2C_Master_Receive_DMA(handle_, dev_addr_, data, size);
        default:
            return HAL_ERROR;
        }
    }

    // ========================= HAL回调函数 =========================

    /**
     * @brief I2C Mem写入完成回调
     */
    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2CInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->tx_cbk_ != nullptr)
            {
                ins->tx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief I2C Mem读取完成回调
     */
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2CInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->rx_cbk_ != nullptr)
            {
                ins->rx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief I2C Master发送完成回调
     */
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2CInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->tx_cbk_ != nullptr)
            {
                ins->tx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief I2C Master接收完成回调
     */
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2CInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->rx_cbk_ != nullptr)
            {
                ins->rx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief I2C错误回调
     */
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2CInstance::instance_list_)
        {
            if (ins->handle_ == hi2c)
            {
                LOGERROR("[sal::I2C] error callback, ErrorCode=0x%x", hi2c->ErrorCode);
                return;
            }
        }
    }

} // namespace sal
