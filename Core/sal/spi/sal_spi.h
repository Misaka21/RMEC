#pragma once
#include "spi.h"
#include "xTools.hpp"
#include <functional>
#include <vector>

namespace sal {

    enum class SpiXferType { BLOCK, IT, DMA };

    class SpiInstance {
        friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);

    public:
        using SpiPtr        = SpiInstance*;
        using SpiTxCallback = std::function<void()>;
        using SpiRxCallback = std::function<void()>;

        struct SpiConfig {
            SPI_HandleTypeDef *handle    = nullptr;
            SpiXferType    xfer_type = SpiXferType::BLOCK;
            GPIO_TypeDef      *cs_port   = nullptr;
            uint16_t           cs_pin    = 0;
            SpiTxCallback      tx_cbk    = nullptr;
            SpiRxCallback      rx_cbk    = nullptr;
        };

    private:
        static std::vector<SpiPtr> instance_list_;

        SPI_HandleTypeDef *handle_;
        SpiXferType    xfer_type_;
        GPIO_TypeDef      *cs_port_;
        uint16_t           cs_pin_;
        SpiTxCallback      tx_cbk_;
        SpiRxCallback      rx_cbk_;

    public:
        SpiInstance(const SpiConfig &config);
        ~SpiInstance() = default;

        // 低层传输方法,根据xfer_type调用对应HAL函数
        HAL_StatusTypeDef SpiTransmit(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef SpiReceive(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef SpiTransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout = 1000);

        // 寄存器便捷方法(始终阻塞,带CS控制)
        uint8_t ReadReg(uint8_t reg);
        void    WriteReg(uint8_t reg, uint8_t data);
        void    ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
        void    ReadRegsDMA(uint8_t reg, uint8_t *buf, uint16_t len);

        // CS引脚控制
        void CSSet();
        void CSReset();
    };

} // namespace sal
