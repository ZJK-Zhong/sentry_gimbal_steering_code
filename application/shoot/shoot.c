/**
 * @file shoot.c
 * @brief 基于rm-sentry的DJIMotor和消息中心控制双摩擦轮、拨弹轮
 * @note 三电机均在云台CAN2；裁判热量由底盘USART6接收，再经板间CAN和chassis_feed传入。
 *       保留原摩擦轮/拨盘速度环参数和角速度单位deg/s，无力必须发送零电流而不是速度零闭环。
 */
#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include <math.h>

#define FRICTION_SPEED_DPS 30000.0f // 原rm-sentry摩擦轮设定，相当于5000rpm。
#define LOADER_SPEED_DPS 10000.0f   // 原rm-sentry连发设定。
#define LOADER_JAM_CURRENT 13000.0f // 原rm-sentry卡弹阈值，反馈单位沿用DJIMotor。
#define HEAT_RESERVE 50u            // 原shoot_check的热量余量。

static DJIMotorInstance *friction_l, *friction_r, *loader;
static Subscriber_t *shoot_sub, *chassis_sub;
static Publisher_t *shoot_pub;
static Shoot_Upload_Data_s shoot_feedback;
static uint32_t friction_start_ms, jam_start_ms, reverse_start_ms;
static uint8_t friction_spinning, jam_seen, reversing;

/** @brief 停止三个发射电机并清空启动、堵转计时，下一次使能必须重新等待摩擦轮。 */
static void StopShooter(void)
{
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);
    friction_spinning = jam_seen = reversing = 0;
}

/** @brief 读取反馈是否存在且未超过20ms，在反馈临界区中调用。 */
static uint8_t MotorIsOnline(DJIMotorInstance *motor)
{
    return motor->feedback_seen && (uint32_t)(HAL_GetTick() - motor->last_rx_ms) <= 20u;
}

void ShootInit(void)
{
    // 保留rm-sentry的0.85速度P环、相反安装方向及ID6/7，仅统一到云台CAN2。
    Motor_Init_Config_s friction_config = {
        .can_init_config = {.can_handle = SHOOT_CAN_HANDLE, .tx_id = SHOOT_FRICTION_RIGHT_ID},
        .controller_param_init_config = {
            .speed_PID = {.Kp = 0.85f, .Ki = 0, .Kd = 0,
                .Improve = PID_Integral_Limit, .IntegralLimit = 10000, .MaxOut = 16000}},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL},
        .motor_type = M3508};
    friction_r = DJIMotorInit(&friction_config);
    friction_config.can_init_config.tx_id = SHOOT_FRICTION_LEFT_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_l = DJIMotorInit(&friction_config);

    // 原拨盘2.8速度P环，使用M2006协议输出上限；不复活原代码中未调通的位置环。
    Motor_Init_Config_s loader_config = {
        .can_init_config = {.can_handle = SHOOT_CAN_HANDLE, .tx_id = SHOOT_LOADER_ID},
        .controller_param_init_config = {
            .speed_PID = {.Kp = 2.8f, .Ki = 0, .Kd = 0,
                .Improve = PID_Integral_Limit, .IntegralLimit = 5000, .MaxOut = 10000}},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL},
        .motor_type = M2006};
    loader = DJIMotorInit(&loader_config);
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    chassis_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    StopShooter();
}

void ShootTask(void)
{
    Shoot_Ctrl_Cmd_s cmd = {0};
    Chassis_Upload_Data_s feedback = {0};
    uint8_t command_valid = SubGetMessage(shoot_sub, &cmd);
    uint8_t chassis_valid = SubGetMessage(chassis_sub, &feedback);
    uint32_t now = HAL_GetTick(), irq = __get_PRIMASK();
    __disable_irq();
    shoot_feedback.motors_online = MotorIsOnline(friction_l) &&
        MotorIsOnline(friction_r) && MotorIsOnline(loader);
    float left_speed = fabsf(friction_l->measure.speed_aps);
    float right_speed = fabsf(friction_r->measure.speed_aps);
    float loader_current = fabsf(loader->measure.real_current);
    __set_PRIMASK(irq);

    shoot_feedback.friction_ready = 0;
    shoot_feedback.loader_running = 0;
    shoot_feedback.heat_allowed = chassis_valid && feedback.referee_online &&
        (unsigned)feedback.current_heat + HEAT_RESERVE < feedback.heat_limit;

    if (!command_valid || !shoot_feedback.motors_online ||
        cmd.shoot_mode != SHOOT_ON || cmd.friction_mode != FRICTION_ON)
        StopShooter();
    else
    {
        if (!friction_spinning)
        {
            friction_start_ms = now;
            friction_spinning = 1;
        }
        DJIMotorSetRef(friction_l, FRICTION_SPEED_DPS);
        DJIMotorSetRef(friction_r, FRICTION_SPEED_DPS);
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        // 速度达标且已启动300ms后才允许拨弹；这两项均为运行时实测反馈条件。
        shoot_feedback.friction_ready = (uint32_t)(now - friction_start_ms) >= 300u &&
            left_speed >= FRICTION_SPEED_DPS * 0.8f && right_speed >= FRICTION_SPEED_DPS * 0.8f;
        if (cmd.load_mode == LOAD_BURSTFIRE && shoot_feedback.friction_ready &&
            shoot_feedback.heat_allowed)
        {
            // 原反转解堵逻辑增加100ms持续判定和100ms反转窗口，避免逐周期正反抖动。
            if (loader_current > LOADER_JAM_CURRENT)
            {
                if (!jam_seen) { jam_start_ms = now; jam_seen = 1; }
                if (!reversing && (uint32_t)(now - jam_start_ms) >= 100u)
                { reversing = 1; reverse_start_ms = now; }
            }
            else jam_seen = 0;
            if (reversing && (uint32_t)(now - reverse_start_ms) >= 100u)
            { reversing = 0; jam_seen = 0; }
            DJIMotorSetRef(loader, reversing ? -LOADER_SPEED_DPS : LOADER_SPEED_DPS);
            DJIMotorEnable(loader);
            shoot_feedback.loader_running = 1;
        }
        else
        {
            DJIMotorStop(loader);
            jam_seen = reversing = 0;
        }
    }
    PubPushMessage(shoot_pub, &shoot_feedback);
}
