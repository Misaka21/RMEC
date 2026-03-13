#include "comm_task.hpp"
#include "robot_def.hpp"

#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)

#include "can_comm.hpp"
#include "robot_topics.hpp"
#include "TaskManager.hpp"

static CanComm<BoardCommTxData, BoardCommRxData>* comm;

// Topic readers (各板按需订阅)
#if defined(CHASSIS_BOARD)
static TopicReader<referee::RefereeData>* ref_reader;
#elif defined(GIMBAL_BOARD)
static TopicReader<ChassisCmdData>*       chassis_cmd_reader;
static TopicReader<InsData>*              ins_reader;
#endif

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

#if defined(CHASSIS_BOARD)
            ref_reader         = referee_topic.Subscribe();
#elif defined(GIMBAL_BOARD)
            chassis_cmd_reader = chassis_cmd_topic.Subscribe();
            ins_reader         = ins_topic.Subscribe();
#endif
        },

        .task_func = []() {
            // static 保持上次有效值, 避免 SeqLock 碰撞时发送全零帧
            static BoardCommTxData tx{};

#if defined(CHASSIS_BOARD)
            // 底盘板 → 云台板: 裁判系统数据转发
            referee::RefereeData ref{};
            if (ref_reader->Read(ref)) {
                // 功率热量
                tx.buffer_energy          = ref.power_heat.buffer_energy;
                tx.shooter_17mm_heat      = ref.power_heat.shooter_17mm_barrel_heat;
                tx.shooter_42mm_heat      = ref.power_heat.shooter_42mm_barrel_heat;
                // 机器人状态
                tx.shooter_heat_limit     = ref.robot_status.shooter_barrel_heat_limit;
                tx.shooter_cooling_value  = ref.robot_status.shooter_barrel_cooling_value;
                tx.chassis_power_limit    = ref.robot_status.chassis_power_limit;
                tx.robot_level            = ref.robot_status.robot_level;
                tx.robot_id               = ref.robot_status.robot_id;
                tx.power_output           = ref.robot_status.power_management_output;
                // 允许发弹量
                tx.projectile_allowance_17mm = ref.projectile_allowance.projectile_allowance_17mm;
                tx.projectile_allowance_42mm = ref.projectile_allowance.projectile_allowance_42mm;
                tx.remaining_gold_coin       = ref.projectile_allowance.remaining_gold_coin;
                // 射击信息
                tx.bullet_speed           = ref.shoot_data.initial_speed;
                // 比赛状态
                tx.game_progress          = ref.game_status.game_progress;
                // 增益
                tx.recovery_buff          = ref.buff.recovery_buff;
                tx.cooling_buff           = ref.buff.cooling_buff;
                tx.attack_buff            = ref.buff.attack_buff;
                // 哨兵信息
                tx.sentry_info            = ref.sentry_info.sentry_info;
                tx.sentry_info_2          = ref.sentry_info.sentry_info_2;
            }
#elif defined(GIMBAL_BOARD)
            // 云台板 → 底盘板: 底盘控制命令 + 云台姿态
            ChassisCmdData cmd{};
            if (chassis_cmd_reader->Read(cmd)) {
                tx.chassis_mode = cmd.mode;
                tx.vx           = cmd.vx;
                tx.vy           = cmd.vy;
                tx.wz           = cmd.wz;
                tx.offset_angle = cmd.offset_angle;
            }
            InsData ins{};
            if (ins_reader->Read(ins)) {
                tx.yaw_ins = ins.euler[0];
            }
#endif
            comm->Send(tx);

            // 接收: SeqLock 一致时才发布
            BoardCommRxData rx{};
            if (comm->Recv(rx))
                board_comm_topic.Publish(rx);
        },
    });
}

#else // ONE_BOARD — 不需要双板通信

void CommTaskStart() {}

#endif
