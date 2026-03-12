#include <cstring>
#include "sal_can.h"
#include "sal_dwt.h"

#include "xTools.hpp"
namespace sal
{
    // static uint8_t can_dev_idx = 0; // 已经注册的CAN设备数量

    // 为静态成员变量分配内存
    std::vector<CanInstance::CanPtr> CanInstance::instance_[CAN_DEV_NUM];
    CanInstance::FifoPtr CanInstance::fifo_ptr_[CAN_DEV_NUM];
    uint8_t CanInstance::can1_filter_idx_ = 0;
    uint8_t CanInstance::can2_filter_idx_ = 14;

    CanInstance::CanInstance(const CanConfig &config)
    {
        // sanity check
        if (config.handle == nullptr)
            DEBUG_DEADLOCK("[sal::CanInstance] handle is nullptr");
        if (config.tx_id > 0x7FF || config.rx_id > 0x7FF || config.tx_id == config.rx_id || !config.tx_id || !config.rx_id)
            DEBUG_DEADLOCK("[sal::CanInstance] invalid id");
        if (config.fifo_timeout_us == 0 || config.fifo_timeout_us > 250)
            LOGWARNING("[sal::CanInstance] timeout: %d, tx may fail or RTibility can be affected", config.fifo_timeout_us);
        if (config.rx_cbk == nullptr)
            LOGINFO("[sal::CanInstance] rx_cbk is nullptr, ru sure bout this?");

        handle_ = config.handle;
        tx_id_ = config.tx_id; // @todo 好像存着没什么用?
        rx_id_ = config.rx_id;
        tx_cbk_ = config.tx_cbk;
        rx_cbk_ = config.rx_cbk;
        fifo_timeout_us_ = config.fifo_timeout_us;
        tx_header_.StdId = tx_id_;
        tx_header_.IDE = CAN_ID_STD;
        tx_header_.RTR = CAN_RTR_DATA;
        tx_header_.DLC = MX_CAN_MSG_LEN;

        if (can1_filter_idx_ == 0 && can2_filter_idx_ == 14)
        {
            CanServiceInit(); // 注册第一个实例前初始化CAN服务
            // 初始化tx fifo
            for (uint8_t i = 0; i < CAN_DEV_NUM; i++)
                fifo_ptr_[i] = std::make_shared<LoopQueue<CanMsg>>(CAN_TX_FIFO_SIZE);
        }
        CanAddFilter();

        uint8_t can_dev = handle_->Instance == CAN1 ? 0 : 1; // 用于区分CAN1和CAN2
        // 加入列表,绑定fifo
        instance_[can_dev].push_back(this);
        fifo_bind_ = fifo_ptr_[can_dev];
    }

    // 判断是否使用FIFO,若不使用则将bit置1并循环等待空闲并完成发送
    // 等待FIFO空闲,将msg放入FIFO,
    // FIFO等待超时则返回false,递增busy_cnt
    bool CanInstance::CanTransmit(const CanMsg &msg, uint16_t block_timeout_us)
    {
        // fifo为空,放入后直接启动发送
        if (fifo_bind_->empty())
        {
            fifo_bind_->push(msg);
            fifo_bind_->back().instance = this;
            TxQueueFront(fifo_bind_);
            return true;
        }
        // fifo不为空,在超时时间内等待并入队
        float tstart = DwtInstance::DwtGetTimeline_ms();
        float timeout_ms = static_cast<float>(block_timeout_us) * 0.001f;
        while (DwtInstance::DwtGetTimeline_ms() - tstart < timeout_ms)
        {
            if (fifo_bind_->size() < fifo_bind_->max_size()) {
                fifo_bind_->push(msg);
                fifo_bind_->back().instance = this;
                return true;
            }
        }
        return false;
    }

    void CanInstance::CanServiceInit()
    {
#ifndef CAN2
        HAL_CAN_Start(&hcan);
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
        HAL_CAN_Start(&hcan2);
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif // 兼容双CAN MCU
    }

    void CanInstance::CanAddFilter()
    {
        CAN_FilterTypeDef filter_cfg;
        // bank0-13给can1用,14-27给can2用,当只有can1时,SlaveStartFilterBank参数无效,且只有14个过滤器,都归CAN1使用
        // 当前仅加入标准帧的过滤器,且仅使用每个filter的低16位,最多注册28个规则(28个instance)
        // 实际上若使用16位模式,每个filter支持2个规则,最多注册2*28=56个规则,若要增加filter数量,同时可以使用FilterIdHigh和FilterMaskIdHigh/FilterMaskIdLow
        // 具体请查阅对应型号STM32的参考手册
        filter_cfg.FilterMode = CAN_FILTERMODE_IDLIST;                              // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
        filter_cfg.FilterScale = CAN_FILTERSCALE_16BIT;                             // 使用16位id模式,这里只用到了低16位(FilterIdHigh没有用到)
        filter_cfg.FilterFIFOAssignment = rx_id_ & 1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1; // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
        filter_cfg.FilterIdLow = rx_id_ << 5;                                       // 过滤器寄存器的低16位,因为使用STDID,所以只有最低11位有效,高5位要填0
        filter_cfg.FilterActivation = CAN_FILTER_ENABLE;                            // 启用过滤器
        filter_cfg.SlaveStartFilterBank = 14;                                       // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)

        filter_cfg.FilterBank = // 有can2的时候,can1和can2的过滤器分开配置
#ifndef CAN2
            can1_filter_idx_++;
#endif
        handle_->Instance == CAN1 ? (can1_filter_idx_++) : (can2_filter_idx_++); // 根据can_handle判断是CAN1还是CAN2,然后自增

        if (can1_filter_idx_ > 13)
            DEBUG_DEADLOCK("[sal::CanInstance] too many instances in CAN1");
        if (can2_filter_idx_ > 27)
            DEBUG_DEADLOCK("[sal::CanInstance] too many instances in CAN2");

        HAL_CAN_ConfigFilter(handle_, &filter_cfg);
    }

    /**
     * @brief 获取发送缓冲队列中的一帧数据
     *
     * @param fifo 发送缓冲队列
     */
    void CanInstance::TxQueueFront(FifoPtr &fifo)
    {
        if (fifo->size())
        {
            CanMsg &msg = fifo->front();
            HAL_CAN_AddTxMessage(msg.instance->handle_, &msg.instance->tx_header_, msg.data, &msg.instance->tx_mailbox_);
        }
    }

    void CanInstance::CanTxFinishCallback(CAN_HandleTypeDef *hcan)
    {
        uint8_t can_dev = hcan->Instance == CAN1 ? 0 : 1; // 用于区分CAN1和CAN2

        // 从发送缓冲队列中删除已经发送完成的消息
        CanInstance *ins = fifo_ptr_[can_dev]->pop().instance;
        if (ins->tx_cbk_)
            ins->tx_cbk_();

        TxQueueFront(fifo_ptr_[can_dev]);
        // @todo 可以做条件编译小优化,如果只有CAN1直接调用
    }

    /**
     * @brief rxfifo回调函数,用于处理接收到的消息
     *
     * @param hcan 消息来自哪一个can设备
     * @param fifo 消息来自哪一个fifo
     *
     * @todo 考虑替换HAL_CAN_GetRxMessage(),自己实现直接读取寄存器获取接收id,避免二次复制data
     */
    void CanInstance::CanFifoxCallback(CAN_HandleTypeDef *hcan, uint32_t fifo)
    {
        // 临时变量,用于存储接收到的消息
        uint8_t rx_tmp[8] = {0};
        CAN_RxHeaderTypeDef rx_header_tmp;
        HAL_CAN_GetRxMessage(hcan, fifo, &rx_header_tmp, rx_tmp);
        uint8_t len = rx_header_tmp.DLC; // 减少访存次数

        uint8_t can_dev = hcan->Instance == CAN1 ? 0 : 1; // 用于区分CAN1和CAN2
        for (auto &instance : instance_[can_dev])
        {
            if (instance->rx_id_ == rx_header_tmp.StdId)
            {
                memcpy(instance->rx_data_, rx_tmp, len);
                if (instance->rx_cbk_)
                    instance->rx_cbk_(len);
                return;
            }
        }
    }

} // !sal

// extern "C" 转发: 覆盖 HAL __weak 符号, 转发到 namespace sal 内的实现
extern "C" {
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    sal::CanInstance::CanTxFinishCallback(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    sal::CanInstance::CanTxFinishCallback(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    sal::CanInstance::CanTxFinishCallback(hcan);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    sal::CanInstance::CanFifoxCallback(hcan, CAN_RX_FIFO0);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    sal::CanInstance::CanFifoxCallback(hcan, CAN_RX_FIFO1);
}
}
