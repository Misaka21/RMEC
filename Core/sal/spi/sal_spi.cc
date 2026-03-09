#include <cstring>
#include "sal_spi.h"

namespace sal {

    std::vector<SPIInstance::SPIPtr> SPIInstance::instance_list_;

    /**
     * @brief SPI实例构造函数
     *
     * @param config SPI配置结构体
     */
    SPIInstance::SPIInstance(const SPIConfig &config)
        : handle_(config.handle),
          xfer_type_(config.xfer_type),
          cs_port_(config.cs_port),
          cs_pin_(config.cs_pin),
          tx_cbk_(config.tx_cbk),
          rx_cbk_(config.rx_cbk)
    {
        if (handle_ == nullptr)
            DEBUG_DEADLOCK("[sal::SPI] handle is nullptr");

        instance_list_.emplace_back(std::shared_ptr<SPIInstance>(this));
    }

    /**
     * @brief CS引脚拉高(释放)
     */
    void SPIInstance::CSSet()
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }

    /**
     * @brief CS引脚拉低(选中)
     */
    void SPIInstance::CSReset()
    {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    }

    /**
     * @brief SPI发送,根据xfer_type调用对应HAL函数
     */
    HAL_StatusTypeDef SPIInstance::SPITransmit(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case SPI_XFER_BLOCK:
            return HAL_SPI_Transmit(handle_, data, size, timeout);
        case SPI_XFER_IT:
            return HAL_SPI_Transmit_IT(handle_, data, size);
        case SPI_XFER_DMA:
            return HAL_SPI_Transmit_DMA(handle_, data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief SPI接收,根据xfer_type调用对应HAL函数
     */
    HAL_StatusTypeDef SPIInstance::SPIReceive(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case SPI_XFER_BLOCK:
            return HAL_SPI_Receive(handle_, data, size, timeout);
        case SPI_XFER_IT:
            return HAL_SPI_Receive_IT(handle_, data, size);
        case SPI_XFER_DMA:
            return HAL_SPI_Receive_DMA(handle_, data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief SPI全双工收发,根据xfer_type调用对应HAL函数
     */
    HAL_StatusTypeDef SPIInstance::SPITransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout)
    {
        switch (xfer_type_)
        {
        case SPI_XFER_BLOCK:
            return HAL_SPI_TransmitReceive(handle_, tx_data, rx_data, size, timeout);
        case SPI_XFER_IT:
            return HAL_SPI_TransmitReceive_IT(handle_, tx_data, rx_data, size);
        case SPI_XFER_DMA:
            return HAL_SPI_TransmitReceive_DMA(handle_, tx_data, rx_data, size);
        default:
            return HAL_ERROR;
        }
    }

    /**
     * @brief 读取单个寄存器(始终阻塞,带CS控制)
     *
     * @param reg 寄存器地址
     * @return 读取到的数据
     */
    uint8_t SPIInstance::ReadReg(uint8_t reg)
    {
        uint8_t tx = reg | 0x80;
        uint8_t rx = 0;
        CSReset();
        HAL_SPI_Transmit(handle_, &tx, 1, 1000);
        HAL_SPI_Receive(handle_, &rx, 1, 1000);
        CSSet();
        return rx;
    }

    /**
     * @brief 写入单个寄存器(始终阻塞,带CS控制)
     *
     * @param reg 寄存器地址
     * @param data 写入的数据
     */
    void SPIInstance::WriteReg(uint8_t reg, uint8_t data)
    {
        uint8_t tx[2] = {reg, data};
        CSReset();
        HAL_SPI_Transmit(handle_, tx, 2, 1000);
        CSSet();
    }

    /**
     * @brief 连续读取多个寄存器(始终阻塞,带CS控制)
     *
     * @param reg 起始寄存器地址
     * @param buf 接收缓冲区
     * @param len 读取字节数
     */
    void SPIInstance::ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        uint8_t tx = reg | 0x80;
        CSReset();
        HAL_SPI_Transmit(handle_, &tx, 1, 1000);
        HAL_SPI_Receive(handle_, buf, len, 1000);
        CSSet();
    }

    /**
     * @brief DMA方式连续读取多个寄存器
     *        发送寄存器地址使用阻塞方式,接收数据使用DMA
     *        CS引脚在DMA接收完成回调中拉高
     *
     * @param reg 起始寄存器地址
     * @param buf 接收缓冲区
     * @param len 读取字节数
     */
    void SPIInstance::ReadRegsDMA(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        uint8_t tx = reg | 0x80;
        CSReset();
        HAL_SPI_Transmit(handle_, &tx, 1, 1000);
        HAL_SPI_Receive_DMA(handle_, buf, len);
    }

    // ========================= HAL回调函数 =========================

    /**
     * @brief SPI发送完成回调
     */
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
    {
        for (auto &ins : SPIInstance::instance_list_)
        {
            if (ins->handle_ == hspi && ins->tx_cbk_ != nullptr)
            {
                ins->tx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief SPI接收完成回调,ReadRegsDMA完成后在此拉高CS
     */
    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
    {
        for (auto &ins : SPIInstance::instance_list_)
        {
            if (ins->handle_ == hspi)
            {
                ins->CSSet();
                if (ins->rx_cbk_ != nullptr)
                    ins->rx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief SPI全双工收发完成回调
     */
    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
    {
        for (auto &ins : SPIInstance::instance_list_)
        {
            if (ins->handle_ == hspi)
            {
                ins->CSSet();
                if (ins->rx_cbk_ != nullptr)
                    ins->rx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief SPI错误回调
     */
    void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
    {
        for (auto &ins : SPIInstance::instance_list_)
        {
            if (ins->handle_ == hspi)
            {
                LOGERROR("[sal::SPI] error callback, ErrorCode=0x%x", hspi->ErrorCode);
                return;
            }
        }
    }

} // namespace sal
