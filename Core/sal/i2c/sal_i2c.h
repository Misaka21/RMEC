#pragma once
#include "i2c.h"
#include "xTools.hpp"
#include <functional>
#include <vector>

namespace sal {

    enum class I2cXferType { BLOCK, IT, DMA };

    class I2cInstance {
        friend void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);

    public:
        using I2cPtr        = I2cInstance*;
        using I2cTxCallback = std::function<void()>;
        using I2cRxCallback = std::function<void()>;

        struct I2cConfig {
            I2C_HandleTypeDef *handle    = nullptr;
            uint16_t           dev_addr  = 0;     // 7-bit地址
            I2cXferType    xfer_type = I2cXferType::BLOCK;
            I2cTxCallback      tx_cbk    = nullptr;
            I2cRxCallback      rx_cbk    = nullptr;
        };

    private:
        static std::vector<I2cPtr> instance_list_;

        I2C_HandleTypeDef *handle_;
        uint16_t           dev_addr_;
        I2cXferType    xfer_type_;
        I2cTxCallback      tx_cbk_;
        I2cRxCallback      rx_cbk_;

    public:
        I2cInstance(const I2cConfig &config);
        ~I2cInstance() = default;

        HAL_StatusTypeDef MemRead(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef MemWrite(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef Transmit(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef Receive(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
    };

} // namespace sal
