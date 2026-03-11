#include "comm_task.hpp"
#include "robot_def.hpp"

#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)

#include "can_comm.hpp"
#include "robot_topics.hpp"
#include "TaskManager.hpp"

static CanComm<BoardCommTxData, BoardCommRxData>* comm;

void CommTaskStart() {
    static TaskManager comm_task({
        .name       = "comm",
        .stack_size = 256,
        .priority   = osPriorityNormal,
        .period_ms  = 10,   // 100Hz 通信频率

        .init_func = []() {
            CanCommConfig cfg{};
            cfg.can_handle     = &COMM_CAN_HANDLE;
            cfg.base_tx_id     = COMM_BASE_TX_ID;
            cfg.base_rx_id     = COMM_BASE_RX_ID;
            cfg.daemon_timeout = 100;  // 1s @ 100Hz daemon tick
            comm = new CanComm<BoardCommTxData, BoardCommRxData>(cfg);
        },

        .task_func = []() {
            // 发送: 组装 TxData 并发送
            BoardCommTxData tx{};
            // TODO: 填入实际数据
            comm->Send(tx);

            // 接收: 发布到 Topic 供其他模块消费
            board_comm_topic.Publish(comm->Recv());
        },
    });
}

#else // ONE_BOARD — 不需要双板通信

void CommTaskStart() {}

#endif
