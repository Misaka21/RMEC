#pragma once
#include "can.h"

#include "xTools.hpp"
#include "xStruct.hpp"

#include <memory>
#include <vector>
#include <functional>

#define MX_CAN_INS_NUM 6   // 单条总线最大负载(由于filter设置,最大为14), 推荐负载率为50%
#define MX_CAN_MSG_LEN 8   // 单帧最大数据长度,单位字节
#define CAN_TX_FIFO_SIZE 8 // 发送缓冲区大小,单位帧

#ifdef CAN2
#define CAN_DEV_NUM 2 // 芯片支持的CAN外设数量(听说有3个CAN的芯片?)
#else
#define CAN_DEV_NUM 1
#endif

namespace sal
{
    class CANInstance;
    // 放入fifo中的CAN消息类型
    struct can_msg_t
    {
        // 将CANInstance声明为友元,避免module访问*instance
        friend class CANInstance;
        uint8_t data[8];

    private:
        CANInstance *instance; // 指向发送这一帧的实例,用于发送完成回调函数
    };

    class CANInstance
    {
        // 简化类型定义
    public:
        using CANPtr = std::shared_ptr<CANInstance>;
        using FIFOPtr = std::shared_ptr<loop_queue<can_msg_t>>;
        using CANRxCallback = std::function<void(uint8_t)>; // 参数为这一包的长度
        using CANTxCallback = std::function<void()>;
        // 类共享静态成员

    private:
        // 静态共享指针数组,用于存储所有CAN实例,用于中断回调函数和发送缓冲队列
        static std::vector<CANPtr> instance_[CAN_DEV_NUM];
        // 用于存储CAN1帧的发送缓冲队列,TXCallback会从这里取数据发送
        static FIFOPtr fifo_ptr_[CAN_DEV_NUM];
        // 用于分配过滤器,每次添加新的实例时,会分配一个过滤器并递增此变量,一个can最多分配14个
        static uint8_t can1_filter_idx_, can2_filter_idx_;

        // 每个instance的私有成员
    private:
        /* private data */
        CAN_HandleTypeDef *handle_;
        // 下面两项会在发送时自动设置
        CAN_TxHeaderTypeDef tx_header_;
        uint32_t tx_mailbox_;
        // tx
        uint32_t tx_id_;
        uint8_t tx_len_;
        CANTxCallback tx_cbk_;     // 传输完成回调函数
        uint16_t fifo_timeout_us_; // FIFO超时时间,单位us
        FIFOPtr fifo_bind_;        // 实例绑定到的FIFO
        // rx
        uint32_t rx_id_;
        uint8_t rx_data_[MX_CAN_MSG_LEN];
        CANRxCallback rx_cbk_;
        // 4debug
        uint32_t busy_cnt_;
        float wait_time_;

        /* 初始化设置 */
    private:
        // 启动CAN服务和接收中断
        void CANServiceInit();
        // 构建实例时为接收ID设置过滤规则
        void CANAddFilter();

    public:
        // 初始化配置结构体
        struct CANConfig
        {
            CAN_HandleTypeDef *handle = nullptr;
            uint32_t tx_id = 0;
            uint32_t rx_id = 0;
            CANTxCallback tx_cbk = nullptr;
            CANRxCallback rx_cbk = nullptr;
            uint16_t fifo_timeout_us = 150; // 保证实时性
        };

    public:
        CANInstance(const CANConfig &config);
        ~CANInstance() = default;

        /**
         * @brief 向CAN发送一帧数据,将数据放入FIFO中排队发送
         *
         * @param msg 待发送的数据
         * @param block_timeout_us 阻塞发送时的超时时间,单位us
         * @return true 成功添加到FIFO
         * @return false 发送超时
         *
         * @attention 即使成功加入发送邮箱,也不代表这一帧一定发送成功,可能发生仲裁失败/总线错误等
         */
        bool CANTransmit(const can_msg_t &msg, uint16_t block_timeout_us = 300);

        // 回调设置,需要在外部调用所以设置为static
        static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifo);

        static void CANTxFinishCallback(CAN_HandleTypeDef *hcan);

        /*用于处理发送和发送完成回调函数的函数*/
    private:
        // 取出FIFO中的一帧数据,并发送
        static void TxQueueFront(FIFOPtr &fifo);
        // 发送完成回调函数,用于判断是否需要调用tx_cb
        static void PopQueueCallback(FIFOPtr &fifo);
    };
}