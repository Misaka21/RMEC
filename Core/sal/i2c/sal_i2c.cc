#include "sal_i2c.h"
#include "log.h"

namespace sal {

    std::vector<I2cInstance::I2cPtr> I2cInstance::instance_list_;

    /**
     * @brief I2C实例构造函数
     *
     * @param config I2C配置结构体
     */
    I2cInstance::I2cInstance(const I2cConfig &config)
        : handle_(config.handle),
          dev_addr_(config.dev_addr << 1), // HAL需要左移1位的地址
          xfer_type_(config.xfer_type),
          tx_cbk_(config.tx_cbk),
          rx_cbk_(config.rx_cbk)
    {
        if (handle_ == nullptr)
            DEBUG_DEADLOCK("[sal::I2C] handle is nullptr");

        // I2C3未配置DMA stream,不支持DMA模式
        if (handle_->Instance == I2C3 && xfer_type_ == I2cXferType::DMA)
            DEBUG_DEADLOCK("[sal::I2C] I2C3 does not support DMA");

        instance_list_.push_back(this);
    }

    /**
     * @brief I2C寄存器读取
     *
     * @param reg 寄存器地址
     * @param data 接收缓冲区
     * @param size 读取字节数
     * @param timeout 超时时间(仅阻塞模式有效)
     */
    HAL_StatusTypeDef I2cInstance::MemRead(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2cXferType::BLOCK:
            return HAL_I2C_Mem_Read(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size, timeout);
        case I2cXferType::IT:
            return HAL_I2C_Mem_Read_IT(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        case I2cXferType::DMA:
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
    HAL_StatusTypeDef I2cInstance::MemWrite(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2cXferType::BLOCK:
            return HAL_I2C_Mem_Write(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size, timeout);
        case I2cXferType::IT:
            return HAL_I2C_Mem_Write_IT(handle_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, data, size);
        case I2cXferType::DMA:
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
    HAL_StatusTypeDef I2cInstance::Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2cXferType::BLOCK:
            return HAL_I2C_Master_Transmit(handle_, dev_addr_, data, size, timeout);
        case I2cXferType::IT:
            return HAL_I2C_Master_Transmit_IT(handle_, dev_addr_, data, size);
        case I2cXferType::DMA:
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
    HAL_StatusTypeDef I2cInstance::Receive(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case I2cXferType::BLOCK:
            return HAL_I2C_Master_Receive(handle_, dev_addr_, data, size, timeout);
        case I2cXferType::IT:
            return HAL_I2C_Master_Receive_IT(handle_, dev_addr_, data, size);
        case I2cXferType::DMA:
            return HAL_I2C_Master_Receive_DMA(handle_, dev_addr_, data, size);
        default:
            return HAL_ERROR;
        }
    }

    // ========================= HAL回调函数 =========================

    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2cInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->tx_cbk_ != nullptr)
            {
                ins->tx_cbk_();
                return;
            }
        }
    }

    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2cInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->rx_cbk_ != nullptr)
            {
                ins->rx_cbk_();
                return;
            }
        }
    }

    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2cInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->tx_cbk_ != nullptr)
            {
                ins->tx_cbk_();
                return;
            }
        }
    }

    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2cInstance::instance_list_)
        {
            if (ins->handle_ == hi2c && ins->rx_cbk_ != nullptr)
            {
                ins->rx_cbk_();
                return;
            }
        }
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
    {
        for (auto &ins : I2cInstance::instance_list_)
        {
            if (ins->handle_ == hi2c)
            {
                LOGERROR("[sal::I2C] error callback, ErrorCode=0x%x", hi2c->ErrorCode);
                return;
            }
        }
    }

} // namespace sal

// extern "C" 转发: 覆盖 HAL __weak 符号, 转发到 namespace sal 内的实现
extern "C" {
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    sal::HAL_I2C_MemTxCpltCallback(hi2c);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    sal::HAL_I2C_MemRxCpltCallback(hi2c);
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    sal::HAL_I2C_MasterTxCpltCallback(hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    sal::HAL_I2C_MasterRxCpltCallback(hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    sal::HAL_I2C_ErrorCallback(hi2c);
}
}
