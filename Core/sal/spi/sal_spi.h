#pragma once
#include "spi.h"
#include "xTools.hpp"
#include <functional>
#include <vector>

namespace sal {

    enum SPI_Xfer_Type_e { SPI_XFER_BLOCK, SPI_XFER_IT, SPI_XFER_DMA };

    class SPIInstance {
        friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
        friend void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);

    public:
        using SPIPtr        = SPIInstance*;
        using SPITxCallback = std::function<void()>;
        using SPIRxCallback = std::function<void()>;

        struct SPIConfig {
            SPI_HandleTypeDef *handle    = nullptr;
            SPI_Xfer_Type_e    xfer_type = SPI_XFER_BLOCK;
            GPIO_TypeDef      *cs_port   = nullptr;
            uint16_t           cs_pin    = 0;
            SPITxCallback      tx_cbk    = nullptr;
            SPIRxCallback      rx_cbk    = nullptr;
        };

    private:
        static std::vector<SPIPtr> instance_list_;

        SPI_HandleTypeDef *handle_;
        SPI_Xfer_Type_e    xfer_type_;
        GPIO_TypeDef      *cs_port_;
        uint16_t           cs_pin_;
        SPITxCallback      tx_cbk_;
        SPIRxCallback      rx_cbk_;

    public:
        SPIInstance(const SPIConfig &config);
        ~SPIInstance() = default;

        // 低层传输方法,根据xfer_type调用对应HAL函数
        HAL_StatusTypeDef SPITransmit(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef SPIReceive(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef SPITransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout = 1000);

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
