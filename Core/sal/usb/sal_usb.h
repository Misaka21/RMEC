#pragma once

// HAL / USB中间件
#include "usbd_cdc_if.h"
// SAL
#include "log.h"
#include "xTools.hpp"
// STL
#include <cstdint>
#include <functional>

/**
 * @brief USB CDC (Virtual COM Port) 虚拟串口模块
 *
 *        封装了STM32的USB Full Speed CDC驱动,提供与上位机通信的接口
 *        USB设备端的波特率/校验位/数据位等由上位机(主机端)决定,设备端不需要关心这些配置
 *
 * @note  1. USB Full Speed模式下单次传输最大为64字节,超出可能丢包
 *        2. USB是单例外设,只允许创建一个UsbInstance(第二次构造会触发DEBUG_DEADLOCK)
 *        3. 接收缓冲区大小为APP_RX_DATA_SIZE(默认2048字节),由usbd_cdc_if.c中定义
 *
 * @attention 此模块修改了CubeMX生成的usbd_cdc_if.c和usbd_cdc_if.h(在USER CODE区域内),
 *            添加了回调函数注册接口CDCInitRxbufferNcallback(),
 *            若使用CubeMX重新生成代码,USER CODE区域内的修改会被保留,
 *            但请务必检查生成后的文件是否完整
 */
namespace sal {

    /**
     * @brief USB CDC单例实例类
     *
     * @note  与CAN/UART等多实例外设不同,USB只有一个物理端口,因此采用单例模式
     *        内部通过静态指针instance_实现单例,构造时自动注册回调到底层CDC驱动
     */
    class UsbInstance {
    public:
        /* ========================= 类型定义 ========================= */

        // 接收回调: 参数为接收缓冲区指针和本次接收的字节数
        // 比参考项目的C函数指针更灵活,可以绑定lambda/成员函数
        using UsbRxCallback = std::function<void(uint8_t *buf, uint16_t len)>;
        // 发送完成回调: 参数为本次发送的字节数
        using UsbTxCallback = std::function<void(uint16_t len)>;

        /**
         * @brief 初始化配置结构体
         *
         * @note rx_cbk: 接收到数据时的回调,在USB中断上下文中执行,不要做耗时操作
         *       tx_cbk: 发送完成的回调(可选),大多数场景下不需要
         */
        struct UsbConfig {
            UsbRxCallback rx_cbk = nullptr; // 接收完成回调,通常必须设置
            UsbTxCallback tx_cbk = nullptr; // 发送完成回调,可选
        };

    private:
        /* ========================= 单例管理 ========================= */
        static UsbInstance *instance_; // 单例指针,构造时设置,全局唯一

        /* ========================= 私有成员 ========================= */
        uint8_t       *rx_buf_;  // 指向usbd_cdc_if.c中的UserRxBufferFS,大小为APP_RX_DATA_SIZE
        UsbRxCallback  rx_cbk_;  // module层注册的接收回调
        UsbTxCallback  tx_cbk_;  // module层注册的发送完成回调

        /* ========================= 底层回调桥接 =========================
         * CDC驱动的回调是C函数指针(UsbCallback类型),无法直接绑定std::function
         * 这两个静态函数作为桥接: C回调 → 静态函数 → instance_的std::function成员
         */
        static void RxCallbackDispatch(uint16_t len);
        static void TxCallbackDispatch(uint16_t len);

    public:
        /**
         * @brief 构造USB CDC实例,完成以下初始化:
         *        1. 检查单例(重复构造会DEBUG_DEADLOCK)
         *        2. 向底层CDC驱动注册收发回调(CDCInitRxbufferNcallback)
         *        3. 获取接收缓冲区指针
         *
         * @param config 配置结构体,包含收发回调函数
         *
         * @attention USB设备在usbd_conf.c的HAL_PCD_MspInit()中会执行软件复位(模拟拔插),
         *            因此构造后上位机会重新枚举设备
         */
        explicit UsbInstance(const UsbConfig &config);
        ~UsbInstance() = default;

        /**
         * @brief 通过USB CDC发送数据
         *
         * @param data 发送数据指针
         * @param len 发送数据长度,以byte计
         * @return USBD_OK(0) 发送成功
         *         USBD_BUSY 上一次发送尚未完成
         *         USBD_FAIL 发送失败
         *
         * @attention USB Full Speed模式下,单个数据包最大为64字节
         *            若需要发送更长的数据,需要在应用层分包
         */
        uint8_t Transmit(uint8_t *data, uint16_t len);

        /**
         * @brief 获取接收缓冲区指针
         *
         * @return uint8_t* 指向usbd_cdc_if.c中的UserRxBufferFS
         *         大小为APP_RX_DATA_SIZE(默认2048字节)
         *
         * @note 通常不需要直接使用此指针,数据会通过rx_cbk回调传递
         *       但某些场景下(如DMA转发)可能需要直接访问缓冲区
         */
        uint8_t *GetRxBuffer() const { return rx_buf_; }
    };

} // namespace sal
