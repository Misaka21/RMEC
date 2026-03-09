#pragma once
#include "i2c.h"
#include "xTools.hpp"
#include <memory>
#include <functional>
#include <vector>

namespace sal {

    enum I2C_Xfer_Type_e { I2C_XFER_BLOCK, I2C_XFER_IT, I2C_XFER_DMA };

    class I2CInstance {
        friend void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *);
        friend void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);

    public:
        using I2CPtr        = std::shared_ptr<I2CInstance>;
        using I2CTxCallback = std::function<void()>;
        using I2CRxCallback = std::function<void()>;

        struct I2CConfig {
            I2C_HandleTypeDef *handle    = nullptr;
            uint16_t           dev_addr  = 0;     // 7-bit地址
            I2C_Xfer_Type_e    xfer_type = I2C_XFER_BLOCK;
            I2CTxCallback      tx_cbk    = nullptr;
            I2CRxCallback      rx_cbk    = nullptr;
        };

    private:
        static std::vector<I2CPtr> instance_list_;

        I2C_HandleTypeDef *handle_;
        uint16_t           dev_addr_;
        I2C_Xfer_Type_e    xfer_type_;
        I2CTxCallback      tx_cbk_;
        I2CRxCallback      rx_cbk_;

    public:
        I2CInstance(const I2CConfig &config);
        ~I2CInstance() = default;

        HAL_StatusTypeDef MemRead(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef MemWrite(uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef Transmit(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
        HAL_StatusTypeDef Receive(uint8_t *data, uint16_t size, uint32_t timeout = 1000);
    };

} // namespace sal
