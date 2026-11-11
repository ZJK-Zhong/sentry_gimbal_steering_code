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

static attitude_t imu_snapshot; // 地址固定，作为OTHER_FEED的反馈来源。
static DJIMotorInstance *yaw_motor, *pitch_motor;
static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

attitude_t *get_IMU_data(void) { return &imu_snapshot; }

void GimbalInit(void)
{
    INS_Init(); // 使用原BMI088/QuaternionEKF任务，不引入旧舵轮INS_task。
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {.can_handle = GIMBAL_YAW_CAN_HANDLE, .tx_id = GIMBAL_YAW_MOTOR_ID},
        .controller_param_init_config = {
            .angle_PID = {.Kp = 1.0f, .Ki = 0, .Kd = 1.2f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 200, .MaxOut = 400},
            .speed_PID = {.Kp = 1550, .Ki = 45, .Kd = 0,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000, .MaxOut = 30000},
            .other_angle_feedback_ptr = &imu_snapshot.YawTotalAngle,
            .other_speed_feedback_ptr = &imu_snapshot.Gyro[2]},
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP, .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE},
        .motor_type = GM6020};
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {.can_handle = GIMBAL_PITCH_CAN_HANDLE, .tx_id = GIMBAL_PITCH_MOTOR_ID},
        .controller_param_init_config = {
            .angle_PID = {.Kp = 800, .Ki = 50, .Kd = 150,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 200, .MaxOut = 30000},
            .other_angle_feedback_ptr = &imu_snapshot.Pitch,
            .other_speed_feedback_ptr = &imu_snapshot.Gyro[1]},
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP, .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE},
        .motor_type = GM6020}; // Pitch保留rm-sentry当前实际启用的单角度环。
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
    DJIMotorStop(yaw_motor);
    DJIMotorStop(pitch_motor);
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void GimbalTask(void)
{
    uint8_t command_valid = SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    uint8_t imu_online = INS_GetSnapshot(&imu_snapshot) &&
        isfinite(imu_snapshot.YawTotalAngle) && isfinite(imu_snapshot.Pitch) &&
        isfinite(imu_snapshot.Gyro[1]) && isfinite(imu_snapshot.Gyro[2]);
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint8_t motors_online = yaw_motor->feedback_seen && pitch_motor->feedback_seen &&
        (uint32_t)(HAL_GetTick() - yaw_motor->last_rx_ms) <= 20u &&
        (uint32_t)(HAL_GetTick() - pitch_motor->last_rx_ms) <= 20u;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    __set_PRIMASK(irq);
    gimbal_feedback_data.online = motors_online && imu_online;
    // 首帧未到、任务消息缺失、INS/任一电机过期或目标非法时，两轴均保持零电流。
    if (command_valid && gimbal_feedback_data.online &&
        isfinite(gimbal_cmd_recv.yaw) && isfinite(gimbal_cmd_recv.pitch) &&
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_FREE_MODE))
    {
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
    }
    else
    {
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
    }
    gimbal_feedback_data.gimbal_imu_data = imu_snapshot;
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
