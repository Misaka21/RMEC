#pragma once
// HAL
#include "usart.h"
// SAL
#include "xTools.hpp"
#include "xStruct.hpp"
// STL
#include <memory>
#include <functional>
#include <vector>

#define UART_MX_INS_NUM 3          // 取决于开发板引脚设计与MCU架构,RoboMaster C型开发板为1,3,6
#define UART_MX_RX_BUFFER_SIZE 256 // 接收缓冲区大小 @todo: 是否让接收缓冲区由module提供?
namespace sal
{
    /* 发送状态枚举,用于指示调用发送函数后的结果 */
    enum UART_Tx_State_e
    {
        UART_Tx_ERROR,         // 发送失败,一般为非FIFO模式下上一次发送未结束;也有可能是内存泄露导致返回值被修改
        UART_TX_BLOCK_FINISH,  // 阻塞发送
        UART_TX_BLOCK_TIMEOUT, // 阻塞发送超时
        UART_TX_ONGOING,       // dma/it发送中
        UART_TX_WAITING,       // dma/it已经加入发送队列等待
        UART_TX_BUFF_FULL,     // 发送缓冲区已满
    };
    /* 发送模式枚举 */
    enum UART_Tx_Type_e
    {
        UART_TX_DMA,
        UART_TX_IT,
        UART_TX_BLOCK,
    };
    /* 接收模式枚举 */
    enum UART_Rx_Type_e
    {
        UART_RX_BLOCK_NUM,  // 阻塞接收,注意无法使用回调
        UART_RX_DMA_NUM,    // DMA接收一定字节后进入回调
        UART_RX_IT_NUM,     // IT接收一定字节后进入回调
        UART_RX_BLOCK_IDLE, // 阻塞+空闲,接收一定字节或者触发空闲后返回,无法使用回调
        UART_RX_DMA_IDLE,   // DMA+Idle,接收一定字节或者触发空闲时进入回调
        UART_RX_IT_IDLE,    // IT+Idle,接收一定字节或者触发空闲时进入回调
    };

    class UARTInstance
    {
        // HAL回调函数均设置为友元函数以访问私有变量
        friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
        friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
        friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
        friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

    public:
        // 接收回调函数,返回接收缓冲区指针和接收字节数
        using UARTRecvCallback = std::function<void(uint8_t *, uint16_t)>;
        // 发送回调函数,没有返回值,若需要可自行修改
        using UARTTransCallback = std::function<void()>;
        // 智能指针wrapper
        using UARTPtr = std::shared_ptr<UARTInstance>;

        /* 初始化配置结构体,已经给定默认参数 */
        struct UARTConfig
        {
            UART_HandleTypeDef *handle = nullptr;

            uint16_t rx_size = 0;
            UART_Rx_Type_e rx_type = UART_RX_IT_IDLE;
            UARTRecvCallback rx_cbk = nullptr;

            bool use_fifo = false;
            uint8_t queue_mx_size = 1;
            UART_Tx_Type_e tx_type = UART_TX_IT;
            UARTTransCallback tx_cbk = nullptr;
        };

    private:
        // 所有类实例的指针列表,注意为共享的,不要在外部修改
        static std::vector<UARTPtr> instance_list_;

        UART_HandleTypeDef *handle_;

        uint16_t rx_size_;
        uint8_t rx_buff_[UART_MX_RX_BUFFER_SIZE]; // @todo: 优化为动态分配或由module持有,此处只保存指针以节省内存
        UART_Rx_Type_e rx_type_;
        UARTRecvCallback rx_cbk_;

        bool tx_use_fifo_;
        uint8_t tx_queue_mx_size_;
        loop_queue<uint8_t *> tx_queue_; // 注意此处只保存指针,module需自行维护数据块
        loop_queue<uint16_t> tx_len_queue_;
        UART_Tx_Type_e tx_type_;
        UARTTransCallback tx_cbk_;

        void PopSend(); // 发送队列弹出函数,封装方便调用

    public:
        UARTInstance(const UARTConfig &config);
        ~UARTInstance() = default;

        /**
         * @brief 发送函数,BLOCK/IT/DMA三种模式,使用前需先设置发送模式(或初始化时传入)
         *
         * @attention timeout参数仅在[UART_TX_BLOCK]模式下生效
         * @attention 若为IT/DMA,需保证data在离开调用者的作用域前不会被修改且离开后不会被释放!!!
         * @attention 若使用非阻塞的队列发送,需保证这些数据块不被修改/释放!!!
         *
         * @param data 发送数据指针,注意在发送完成前不要修改/释放
         * @param size 发送数据长度,以byte计
         * @param timeout 发送超时时间,单位ms,仅在[UART_TX_BLOCK]模式下生效,注意默认参数值为无限等待
         * @return UART_Tx_State_e 返回发送状态
         */
        UART_Tx_State_e UARTSend(uint8_t *data, uint16_t size, uint32_t timeout = HAL_MAX_DELAY);

        /**
         * @brief 修改发送设置,若使用发送队列请保证修改前的数据已经发送完成
         *
         * @attention 此函数会终止当前的发送并清空队列(若启用了fifo)
         *
         * @param type UART_Tx_Type_e的枚举值,分别为DMA/IT/BLOCK
         * @param use_fifo 是否使用发送队列,默认为false
         * @param queue_mx_size 发送队列最大长度,默认为0不使用
         */
        void UARTSetSendType(UART_Tx_Type_e type, bool use_fifo = false, uint8_t queue_mx_size = 0);

        /**
         * @brief 在阻塞模式下于timeout时间内接收一定量的数据
         *
         * @param data 接收数据缓冲区指针
         * @param target_size 期望接收的数据量,以byte计
         * @param timeout 超时时间,默认为永久
         * @return uint16_t 实际接收到的数据量,若超时则返回0
         */
        uint16_t UARTRecv(uint8_t *data, uint16_t target_size, uint32_t timeout = HAL_MAX_DELAY);

        /**
         * @brief 修改接收设置,请保证缓冲区数据解析完毕/转移到他处
         *        此函数会停止正在进行的接收,并清空接收缓冲区
         *
         * @param type UART_Rx_Type_e的枚举值,分别为分别为 DMA/IT/BLOCK + NUM/IDLE
         * @param size 期望接收一包的数据量,以byte计
         */
        void UARTSetRecvType(UART_Rx_Type_e type, uint16_t size);

        /**
         * @brief 重启接收服务,仅用于IT和DMA模式.
         *        若使用BLOCK模式,请使用UARTRecv函数,否则此函数进入死循环
         */
        void UARTRestartRecv();

        /**
         * @brief 停止当前正在进行的接收并清空缓冲区,BLOCK/IT/DMA三种模式通用
         *        恢复接收请使用UARTRestartRecv函数(仅用于IT和DMA模式)
         *        BLOCK接收重新通过UARTRecv函数发起
         */
        void UARTStopRecv();

    }; // !class UARTInstance
} // !namespace sal