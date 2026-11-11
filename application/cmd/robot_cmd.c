/**
 * @file robot_cmd.c
 * @brief 云台板的输入/输出编排：采集快照，调用策略，分发目标。
 * @note RobotControl负责模式与解锁，VisionCommand负责视觉有效期，
 *       BoardTransport负责板间收发。本文件不执行运动解算或电机闭环。
 */
#include "robot_cmd.h"
#include "board_transport.h"
#include "message_center.h"
#include "remote_control.h"
#if !CHASSIS_BENCH_TEST
#include "vision_command.h"
#endif
#include <math.h>
#include <string.h>

static Publisher_t *chassis_feedback_publisher; ///< 将底盘CAN回传交给本板发射应用。
#if !CHASSIS_BENCH_TEST
static Publisher_t *gimbal_command_publisher;    ///< 云台目标发布端。
static Publisher_t *shoot_command_publisher;     ///< 发射目标发布端。
static Subscriber_t *gimbal_feedback_subscriber; ///< 本板云台反馈订阅端。
#endif
static RobotControlState_s robot_control_state; ///< 私有策略状态，不读取Watch副本。
GimbalControlDebug_s gimbal_control;            ///< 保留原Ozone观测入口，不参与控制。

/**
 * @brief 复制本轮状态到公开观测结构；异步Watch可能看到复制中的字段。
 * @note 控制路径只读取robot_control_state，写Watch不能改变解锁或目标。
 */
static void PublishControlDebug(void)
{
    gimbal_control.bench_active = CHASSIS_BENCH_TEST;
    gimbal_control.rc_online = robot_control_state.rc_online;
    gimbal_control.chassis_online = robot_control_state.chassis_online;
    gimbal_control.gimbal_online = robot_control_state.gimbal_online;
    gimbal_control.armed = robot_control_state.armed;
    gimbal_control.vision_online = robot_control_state.vision_online;
    gimbal_control.selected_mode = robot_control_state.selected_mode;
    gimbal_control.control_mode = robot_control_state.control_mode;
    gimbal_control.chassis_cmd = robot_control_state.chassis_cmd;
    gimbal_control.gimbal_cmd = robot_control_state.gimbal_cmd;
    gimbal_control.shoot_cmd = robot_control_state.shoot_cmd;
    gimbal_control.chassis_feedback = robot_control_state.chassis_feedback;
}

/**
 * @brief 采集本周期输入，各服务负责自身数据的完整性和新鲜度。
 * @param input 非空输出，每轮先清零，防止失联时残留上一轮的就绪标志。
 */
static void ReadControlInputs(RobotControlInput_s *input)
{
    memset(input, 0, sizeof(*input));
    input->remote_online = RemoteControlSnapshot(&input->remote);
    input->chassis_online = BoardTransportReadFeedback(&input->chassis_feedback);
#if !CHASSIS_BENCH_TEST
    input->vision_online = VisionCommandRead(&input->vision);
    input->gimbal_online = SubGetMessage(gimbal_feedback_subscriber, &input->gimbal_feedback) &&
                           input->gimbal_feedback.online &&
                           isfinite(input->gimbal_feedback.gimbal_imu_data.YawTotalAngle) &&
                           isfinite(input->gimbal_feedback.gimbal_imu_data.Pitch) &&
                           isfinite(input->gimbal_feedback.yaw_motor_single_round_angle);
#endif
}

/** @brief 将策略输出分别送往消息中心和板间服务，不直接发送电机电流帧。 */
static void PublishControlOutputs(void)
{
    PubPushMessage(chassis_feedback_publisher, &robot_control_state.chassis_feedback);
#if !CHASSIS_BENCH_TEST
    PubPushMessage(gimbal_command_publisher, &robot_control_state.gimbal_cmd);
    PubPushMessage(shoot_command_publisher, &robot_control_state.shoot_cmd);
#endif
    BoardTransportSendCommand(&robot_control_state.chassis_cmd, robot_control_state.command_send_immediately);
    PublishControlDebug();
}

void RobotCMDInit(void)
{
    RobotControlInit(&robot_control_state);
    RemoteControlInit(&huart3);
    BoardTransportInit();
    chassis_feedback_publisher = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#if !CHASSIS_BENCH_TEST
    VisionCommandInit();
    gimbal_command_publisher = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_command_publisher = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    gimbal_feedback_subscriber = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
#endif
}

void RobotCMDTask(void)
{
    RobotControlInput_s control_input;
    ReadControlInputs(&control_input);
    RobotControlUpdate(&robot_control_state, &control_input);
    PublishControlOutputs();
}
