/**
 * @file gimbal.c
 * @brief 基于rm-sentry的云台应用：消息中心目标、INS快照和DJIMotor闭环
 * @note PID及反向目标约定取自rm-sentry；仅CAN、ID和yaw机械归中值按旧舵轮硬件适配。
 *       机器人任务设置目标，原MotorControlTask计算PID并统一发送电流。
 */
#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include <math.h>

#define GIMBAL_MOTOR_TIMEOUT_MS 20u ///< 两轴反馈最大有效期，ms。

static attitude_t imu_snapshot;                   ///< 地址固定，作为OTHER_FEED反馈来源，不可改为任务栈变量。
static DJIMotorInstance *gimbal_yaw_motor;        ///< Yaw电机，由本应用设目标、MotorTask执行闭环。
static DJIMotorInstance *gimbal_pitch_motor;      ///< Pitch电机，保留原单角度环。
static Publisher_t *gimbal_feedback_publisher;    ///< gimbal_feed发布端，供CMD读取姿态及就绪状态。
static Subscriber_t *gimbal_command_subscriber;   ///< gimbal_cmd订阅端，不访问CMD私有状态。
static Gimbal_Upload_Data_s gimbal_feedback_data; ///< Ozone可整体展开，含本板IMU和执行层在线结果。
static Gimbal_Ctrl_Cmd_s gimbal_command;          ///< 本轮目标，缺失时不执行旧目标。

attitude_t *get_IMU_data(void)
{
    return &imu_snapshot;
}

void GimbalInit(void)
{
    INS_Init(); // 使用原BMI088/QuaternionEKF任务，不引入旧舵轮INS_task。
    Motor_Init_Config_s yaw_config = {
        .can_init_config =
            {
                .can_handle = GIMBAL_YAW_CAN_HANDLE,
                .tx_id = GIMBAL_YAW_MOTOR_ID,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 1.0f,
                        .Ki = 0,
                        .Kd = 1.2f,
                        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 200,
                        .MaxOut = 400,
                    },
                .speed_PID =
                    {
                        .Kp = 1550,
                        .Ki = 45,
                        .Kd = 0,
                        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 10000,
                        .MaxOut = 30000,
                    },
                .other_angle_feedback_ptr = &imu_snapshot.YawTotalAngle,
                .other_speed_feedback_ptr = &imu_snapshot.Gyro[2],
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            },
        .motor_type = GM6020,
    };
    Motor_Init_Config_s pitch_config = {
        .can_init_config =
            {
                .can_handle = GIMBAL_PITCH_CAN_HANDLE,
                .tx_id = GIMBAL_PITCH_MOTOR_ID,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = 800,
                        .Ki = 50,
                        .Kd = 150,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 200,
                        .MaxOut = 30000,
                    },
                .other_angle_feedback_ptr = &imu_snapshot.Pitch,
                .other_speed_feedback_ptr = &imu_snapshot.Gyro[1],
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = OTHER_FEED,
                .speed_feedback_source = OTHER_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            },
        .motor_type = GM6020,
    }; ///< Pitch保留rm-sentry当前实际启用的单角度环。
    gimbal_yaw_motor = DJIMotorInit(&yaw_config);
    gimbal_pitch_motor = DJIMotorInit(&pitch_config);
    DJIMotorStop(gimbal_yaw_motor);
    DJIMotorStop(gimbal_pitch_motor);
    gimbal_feedback_publisher = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_command_subscriber = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 采集固定地址的INS快照与两轴反馈，更新执行层就绪状态。
 * @note 反馈临界区避免读取中途被CAN回调修改；不保证两电机同时采样。
 */
static void UpdateGimbalFeedback(void)
{
    uint8_t imu_online = INS_GetSnapshot(&imu_snapshot) && isfinite(imu_snapshot.YawTotalAngle) &&
                         isfinite(imu_snapshot.Pitch) && isfinite(imu_snapshot.Gyro[1]) &&
                         isfinite(imu_snapshot.Gyro[2]);
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    uint8_t motors_online = gimbal_yaw_motor->feedback_seen && gimbal_pitch_motor->feedback_seen &&
                            (uint32_t)(HAL_GetTick() - gimbal_yaw_motor->last_rx_ms) <= GIMBAL_MOTOR_TIMEOUT_MS &&
                            (uint32_t)(HAL_GetTick() - gimbal_pitch_motor->last_rx_ms) <= GIMBAL_MOTOR_TIMEOUT_MS;
    gimbal_feedback_data.yaw_motor_single_round_angle = gimbal_yaw_motor->measure.angle_single_round;
    __set_PRIMASK(interrupt_mask);
    gimbal_feedback_data.online = motors_online && imu_online;
    gimbal_feedback_data.gimbal_imu_data = imu_snapshot;
}

/** @brief 判断姿态模式与目标是否可执行；消息存在性由任务入口单独检查。 */
static uint8_t IsGimbalTargetValid(void)
{
    return gimbal_feedback_data.online && isfinite(gimbal_command.yaw) && isfinite(gimbal_command.pitch) &&
           (gimbal_command.gimbal_mode == GIMBAL_GYRO_MODE || gimbal_command.gimbal_mode == GIMBAL_FREE_MODE);
}

void GimbalTask(void)
{
    uint8_t command_valid = SubGetMessage(gimbal_command_subscriber, &gimbal_command);
    UpdateGimbalFeedback();
    // 首帧未到、任务消息缺失、INS/任一电机过期或目标非法时，两轴均保持零电流。
    if (command_valid && IsGimbalTargetValid())
    {
        DJIMotorSetRef(gimbal_yaw_motor, gimbal_command.yaw);
        DJIMotorSetRef(gimbal_pitch_motor, gimbal_command.pitch);
        DJIMotorEnable(gimbal_yaw_motor);
        DJIMotorEnable(gimbal_pitch_motor);
    }
    else
    {
        DJIMotorStop(gimbal_yaw_motor);
        DJIMotorStop(gimbal_pitch_motor);
    }
    PubPushMessage(gimbal_feedback_publisher, &gimbal_feedback_data);
}
