#include "sal_usart.h"
#include <cstring>

namespace sal
{ // 静态类成员变量需要在类外部为其分配空间进行初始化
    std::vector<UartInstance::UartPtr> UartInstance::instance_list_;

    UartInstance::UartInstance(const UartConfig &config)
    {
        // sanity check, tx&rx type运行时再检查
        if (config.handle == nullptr)
            DEBUG_DEADLOCK("[sal::UartInstance] check your handle, it's nullptr");
        if (config.tx_type == UartTxType::BLOCK && config.use_fifo)
            DEBUG_DEADLOCK("[sal::UartInstance] check your tx_type, it's UartTxType::BLOCK but use_fifo is true");
        if (config.use_fifo && config.queue_mx_size == 0)
            DEBUG_DEADLOCK("[sal::UartInstance] check your queue_mx_size, it's 0");
        if (config.rx_size == 0)
            LOGWARNING("[sal::UartInstance] check your rx_size, it's 0");
        if (config.rx_cbk == nullptr) // 不过发送没有回调比较有可能,就不提示了
            LOGWARNING("[sal::UartInstance] check your rx_cbk, it's nullptr");

        // 初始化成员变量
        handle_ = config.handle;

        rx_size_ = config.rx_size;
        rx_type_ = config.rx_type;
        rx_cbk_ = config.rx_cbk;

        tx_use_fifo_ = config.use_fifo;
        tx_queue_mx_size_ = tx_use_fifo_ ? config.queue_mx_size : 1; // 非fifo模式下,队列长度设置为一1方便代码复用
        tx_queue_.resize(tx_queue_mx_size_);                         // 初始化发送队列
        tx_type_ = config.tx_type;
        tx_cbk_ = config.tx_cbk;

        // 将实例加入列表
        instance_list_.push_back(this);
        if (instance_list_.size() > UART_MX_INS_NUM)
            DEBUG_DEADLOCK("[sal::UartInstance] you exceed the max instance num");
    }

    /* 私有函数用于提高可读性.将头部的数据取出发送 */
    void UartInstance::PopSend()
    {
        if (tx_queue_.size())
        {
            if (tx_type_ == UartTxType::DMA)
                HAL_UART_Transmit_DMA(handle_, tx_queue_.pop(), tx_len_queue_.pop());
            else if (tx_type_ == UartTxType::IT)
                HAL_UART_Transmit_IT(handle_, tx_queue_.pop(), tx_len_queue_.pop());
            else
                DEBUG_DEADLOCK("[sal::PopSend] UartTxType::BLOCK is invalid in fifo mode");
        }
    }

    UartTxState UartInstance::UartSend(uint8_t *data, uint16_t size, uint32_t timeout)
    {
        // sanity check
        if (data == nullptr || size == 0)
            DEBUG_DEADLOCK("[sal::UartSend] check your send param, data is nullptr or size is 0");
        if (tx_type_ != UartTxType::BLOCK && tx_type_ != UartTxType::DMA && tx_type_ != UartTxType::IT)
            DEBUG_DEADLOCK("[sal::UartSend] check your tx_type_, it's invalid");

        // blocking mode
        if (tx_type_ == UartTxType::BLOCK)
        {
            if (HAL_UART_Transmit(handle_, data, size, timeout) == HAL_TIMEOUT)
                return UartTxState::BLOCK_TIMEOUT;
            else
                return UartTxState::BLOCK_FINISH;
        }

        // non-blocking: DMA | IT
        if ((tx_use_fifo_ && tx_queue_.size() < tx_queue_mx_size_) ||
            !tx_use_fifo_) // 使用fifo,队列未满 || 不使用fifo
        {
            tx_queue_.push(data);
            tx_len_queue_.push(size);
            if (tx_queue_.size() == 1) // 队列中只有刚刚新增的元素,直接发送
            {
                PopSend();
                return UartTxState::TX_ONGOING;
            }
            else
                return UartTxState::TX_WAITING; // 只加入队列,等待上一帧结束后在回调中发送
        }
        else // (tx_use_fifo_ && tx_queue_.size() >= tx_queue_mx_size_) 使用fifo但队列已满
        {
            LOGWARNING("[sal::UartSend] tx queue is full");
            return UartTxState::BUFF_FULL;
        }
    }

    /* 是否将发送停止分离出来供用户调用? 当前似乎没有必要 */
    void UartInstance::UartSetSendType(UartTxType type, bool use_fifo, uint8_t queue_mx_size)
    {
        if (type == UartTxType::BLOCK && use_fifo)
            DEBUG_DEADLOCK("[sal::UartSetSendType] tx blocking can't use FIFO");
        LOGINFO("[sal::UartSetSendType] change send type to %d, use_fifo %d, queue_sz %d", type, use_fifo, queue_mx_size);
        tx_type_ = type;
        tx_use_fifo_ = use_fifo;
        HAL_UART_AbortTransmit(handle_); // 取消当前发送
        tx_queue_.clear();               // 清空队列
        tx_len_queue_.clear();
        tx_queue_mx_size_ = queue_mx_size;
    }

    void UartInstance::UartSetRecvType(UartRxType type, uint16_t size)
    {
        LOGINFO("[sal::UartSetRecvType] change recv type to %d, size %d,beware not to exceed the buffer size", type, size);
        rx_type_ = type;
        rx_size_ = size;
        UartStopRecv();
        if (type != UartRxType::BLOCK_IDLE && type != UartRxType::BLOCK_NUM)
            UartRestartRecv(); // 非阻塞模式,重启接收服务
    }

    uint16_t UartInstance::UartRecv(uint8_t *data, uint16_t target_size, uint32_t timeout)
    {
        if (data == nullptr || target_size == 0)
            DEBUG_DEADLOCK("[sal::UartRecv] check your recv param, data is nullptr or size is 0");

        if (rx_type_ == UartRxType::BLOCK_NUM)
        {
            if (HAL_UART_Receive(handle_, data, target_size, timeout) == HAL_OK)
                return target_size;
        }
        else if (rx_type_ == UartRxType::BLOCK_IDLE)
        {
            uint16_t real_rx_size = 0;
            if (HAL_UARTEx_ReceiveToIdle(handle_, data, target_size, &real_rx_size, timeout) == HAL_OK)
                return real_rx_size;
        }
        else
            DEBUG_DEADLOCK("[sal::UartRecv] rx_type_ isnt in BLOCKING");

        return 0; // recv failed
    }

    // @todo: 是否让接收缓冲区由module提供?
    void UartInstance::UartStopRecv()
    {
        HAL_UART_AbortReceive(handle_);
        memset(rx_buff_, 0, UART_MX_RX_BUFFER_SIZE); // 清空接收缓冲区
        LOGINFO("[sal::UartStopRecv] reception has been stopped");
    }

    /* 接收完成回调使用或发生异常回调使用,一般用于连续异步接收 */
    void UartInstance::UartRestartRecv()
    {
        switch (rx_type_)
        {
        case UartRxType::DMA_IDLE:
            HAL_UARTEx_ReceiveToIdle_DMA(handle_, rx_buff_, rx_size_);
            __HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
            break;
        case UartRxType::IT_IDLE:
            HAL_UARTEx_ReceiveToIdle_IT(handle_, rx_buff_, rx_size_);
            break;
        case UartRxType::DMA_NUM:
            HAL_UART_Receive_DMA(handle_, rx_buff_, rx_size_);
            __HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
        case UartRxType::IT_NUM:
            HAL_UART_Receive_IT(handle_, rx_buff_, rx_size_);
            break;
        default:
            DEBUG_DEADLOCK("[sal::UartRestartRecv] check your rx_type_");
            break;
        }
    }

    /**
     * @brief 接收完成回调函数, 设定的接收类型为UartRxType::DMA_IDLE | UartRxType::IT_IDLE
     *
     * @note HAL_UART_RxCpltCallback也会进入此函数,两者的差别在于IDLE接收可能不会收到全部的数据
     *
     * @param huart uart句柄
     * @param Size 这一包接收到的数据大小
     */
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        for (auto ins : sal::UartInstance::instance_list_)
        {
            if (ins->handle_ == huart && ins->rx_cbk_ != nullptr)
            {
                ins->rx_cbk_(ins->rx_buff_, Size);
                ins->UartRestartRecv();
                break;
            }
        }
    }

    /**
     * @brief 接收完成回调函数, 设定的接收类型为UartRxType::DMA_NUM | UartRxType::IT_NUM
     *        在接收完成后会进入此函数,注意阻塞发送不会进入此函数
     *
     * @attention 这里偷懒了,直接调用了HAL_UARTEx_RxEventCallback.
     *
     * @param huart uart句柄
     */
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
        sal::HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize); // 一定是收到了全部的数据
    }

    /**
     * @brief 发送完成回调函数,若使用了发送队列,则会检查队列是否为空
     *        不为空则启动一次新的发送
     *
     * @param huart uart句柄
     */
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
    {
        for (auto ins : UartInstance::instance_list_)
        {
            if (ins->handle_ == huart)
            {
                if (ins->tx_use_fifo_)
                    if (ins->tx_queue_.size()) // 如果队列不为空,继续发送
                        ins->PopSend();        // 没有使用fifo则在启动第一次发送时就已经清空队列
                if (ins->tx_cbk_ != nullptr)
                    ins->tx_cbk_();
                return;
            }
        }
    }

    /**
     * @brief 若是中断接收则会重新启动接收;
     *        若是发送错误则会检查发送队列是否为空,不为空则启动一次新的发送
     *
     * @todo 判断是接收函数发送错误并继续发送或?
     *
     * @param huart
     */
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        for (auto ins : UartInstance::instance_list_)
        {
            if (ins->handle_ == huart)
            {
                LOGWARNING("[sal::UARTErrCbk] uart error");
                if (ins->rx_type_ != UartRxType::BLOCK_IDLE &&
                    ins->rx_type_ != UartRxType::BLOCK_NUM)
                    ins->UartRestartRecv(); // 不是阻塞,重启接收服务
                // 若有需要可添加module的接收/发送错误回调
                // 当前只处理recv error
            }
        }
    }
} // !sal
