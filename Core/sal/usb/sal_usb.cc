#include "sal_usb.h"

namespace sal {

    // 单例指针初始化为nullptr,构造第一个实例时设置
    USBInstance *USBInstance::instance_ = nullptr;

    /**
     * @brief USB CDC实例构造函数
     *
     * @param config 配置结构体
     *
     * @note 初始化流程:
     *       1. 单例检查: 若instance_不为空则进入死循环(USB只有一个物理端口)
     *       2. 设置单例指针
     *       3. 调用CDCInitRxbufferNcallback()注册静态桥接函数到底层CDC驱动,
     *          同时获取接收缓冲区指针(UserRxBufferFS)
     *       4. 通过RTT打印初始化成功日志
     */
    USBInstance::USBInstance(const USBConfig &config)
        : rx_buf_(nullptr),
          rx_cbk_(config.rx_cbk),
          tx_cbk_(config.tx_cbk)
    {
        // 单例保护: USB只有一个端口,不允许创建第二个实例
        if (instance_ != nullptr)
            DEBUG_DEADLOCK("[sal::USB] only one instance allowed");

        instance_ = this;

        // 向底层CDC驱动注册回调,并获取接收缓冲区指针
        // CDCInitRxbufferNcallback()定义在usbd_cdc_if.c中(USER CODE区域)
        // 参数顺序: 发送完成回调, 接收完成回调
        rx_buf_ = CDCInitRxbufferNcallback(TxCallbackDispatch, RxCallbackDispatch);

        LOGINFO("[sal::USB] init success");
    }

    /**
     * @brief 通过USB CDC发送数据,内部调用CDC_Transmit_FS()
     *
     * @note CDC_Transmit_FS()定义在usbd_cdc_if.c中,
     *       若上一次发送尚未完成(TxState != 0)会返回USBD_BUSY
     */
    uint8_t USBInstance::Transmit(uint8_t *data, uint16_t len)
    {
        return CDC_Transmit_FS(data, len);
    }

    // ========================= 底层回调桥接函数 =========================
    // 这两个静态函数被注册到usbd_cdc_if.c中,在USB中断上下文中被调用
    // 作用是将C函数指针回调转发到C++ std::function成员

    /**
     * @brief 接收回调桥接: CDC底层收到数据后调用此函数
     *
     * @param len 本次接收到的数据字节数
     *
     * @note 数据存放在rx_buf_(即UserRxBufferFS)中,
     *       此函数将缓冲区指针和长度一起传递给module层的rx_cbk_
     */
    void USBInstance::RxCallbackDispatch(uint16_t len)
    {
        if (instance_ && instance_->rx_cbk_)
            instance_->rx_cbk_(instance_->rx_buf_, len);
    }

    /**
     * @brief 发送完成回调桥接: CDC底层发送完成后调用此函数
     *
     * @param len 本次发送完成的数据字节数
     */
    void USBInstance::TxCallbackDispatch(uint16_t len)
    {
        if (instance_ && instance_->tx_cbk_)
            instance_->tx_cbk_(len);
    }

} // namespace sal
