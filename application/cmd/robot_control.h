/**
 * @file robot_control.h
 * @brief 整车模式与解锁策略：输入本轮快照，输出三类应用目标。
 * @note 不读取串口、CAN、消息中心或系统时钟，不直接操作电机。
 */
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "robot_def.h"
#include "remote_control.h"

/**
 * @brief 双板系统的整车操作模式，数值仅用于本板调试。
 * @note 左下：右下无力、右中摩擦、右上发射；左中右下比赛；
 *       左上：右下移动、右中小陀螺、右上视觉发射。其余组合安全停机。
 */
typedef enum
{
    ROBOT_CONTROL_MODE_SAFE = 0,
    ROBOT_CONTROL_MODE_COMPETITION,
    ROBOT_CONTROL_MODE_GIMBAL_FIRE,
    ROBOT_CONTROL_MODE_MOVE,
    ROBOT_CONTROL_MODE_SPIN,
    ROBOT_CONTROL_MODE_FRICTION_ONLY,
    ROBOT_CONTROL_MODE_FIRE,
    ROBOT_CONTROL_MODE_ZERO_FORCE,
} RobotControlMode_e;

/**
 * @brief CMD采集的一轮输入；在线标志必须对应当前快照，不能沿用旧标志。
 * @note 遥控和视觉的新鲜度由采集层检查；云台反馈包含本板电机/INS联锁结果。
 */
typedef struct
{
    RC_ctrl_t remote;                       ///< 遥控器通道与拨杆快照。
    Vision_Recv_s vision;                   ///< 视觉命令，角度为rad。
    Gimbal_Upload_Data_s gimbal_feedback;   ///< 本板INS姿态deg、角速度rad/s及Yaw编码器角度。
    Chassis_Upload_Data_s chassis_feedback; ///< 底盘回传，速度mm/s、rad/s。
    uint8_t remote_online;                  ///< 遥控帧有效且未超时。
    uint8_t vision_online;                  ///< 视觉帧有效且未超时。
    uint8_t gimbal_online;                  ///< 云台反馈完整、数值合法且执行层就绪。
    uint8_t chassis_online;                 ///< 板间反馈有效，不代表底盘八电机均就绪。
} RobotControlInput_s;

/**
 * @brief 一套独立控制上下文，由CMD持有，外部仅通过更新接口驱动。
 * @note 目标每轮重新生成；连续云台目标和巡查方向跨周期保存。此结构不经CAN发送。
 */
typedef struct
{
    uint8_t rc_online;                      ///< 保留调试字段，表示遥控输入有效。
    uint8_t chassis_online;                 ///< 底盘反馈链路有效。
    uint8_t gimbal_online;                  ///< 正常云台版的两轴/INS联锁通过。
    uint8_t armed;                          ///< 双下解锁；输入失效或拨轮急停撤销。
    uint8_t vision_online;                  ///< 视觉命令可供自动模式使用。
    RobotControlMode_e selected_mode;       ///< 拨杆请求的整车模式。
    RobotControlMode_e control_mode;        ///< 联锁和联调限制后的实际模式。
    Chassis_Ctrl_Cmd_s chassis_cmd;         ///< 本轮底盘目标，mm/s、rad/s、deg。
    Gimbal_Ctrl_Cmd_s gimbal_cmd;           ///< 本轮云台目标，deg。
    Shoot_Ctrl_Cmd_s shoot_cmd;             ///< 本轮发射请求，最终联锁由发射应用执行。
    Chassis_Upload_Data_s chassis_feedback; ///< 本轮底盘反馈，仅参与整车策略。
    float yaw_target_deg;                   ///< 连续Yaw目标，保留rm-sentry反向约定。
    float pitch_target_deg;                 ///< Pitch目标，受机械软件限幅。
    int8_t patrol_pitch_direction;          ///< 视觉无目标时俯仰巡查方向，+1或-1。
    uint8_t was_active;                     ///< 上一周期是否处于非安全、非无力模式。
    uint8_t command_send_immediately;       ///< 模式变化或保护停机时请求立即发送。
} RobotControlState_s;

/**
 * @brief 初始化一套整车控制上下文，默认未解锁、目标为零。
 * @param state 已分配的非空上下文；不得与正在运行的更新调用并发。
 */
void RobotControlInit(RobotControlState_s *state);

/**
 * @brief 按左右拨杆编码选择请求模式；不处理解锁和在线联锁。
 * @param left_switch 左开关编码，RC_SW_DOWN/MID/UP。
 * @param right_switch 右开关编码，RC_SW_DOWN/MID/UP。
 * @return 请求模式；非法编码返回SAFE。
 */
RobotControlMode_e SelectRobotControlMode(uint8_t left_switch, uint8_t right_switch);

/**
 * @brief 执行一轮解锁、目标复位、模式策略和发送边沿判定。
 * @param state 非空且已初始化的上下文，状态只属于当前调用方。
 * @param input 当前周期的非空输入快照；本函数不保存其指针。
 * @note CHASSIS_BENCH_TEST只允许手动和小陀螺，不伪造云台在线。
 *       不读取时间、不判断传输超时、不发布话题，便于单独验证策略。
 */
void RobotControlUpdate(RobotControlState_s *state, const RobotControlInput_s *input);

#endif
