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

#define FRICTION_SPEED_DPS 30000.0f     // 原rm-sentry摩擦轮设定，相当于5000rpm。
#define LOADER_SPEED_DPS 10000.0f       // 原rm-sentry连发设定。
#define LOADER_JAM_CURRENT 13000.0f     // 原rm-sentry卡弹阈值，反馈单位沿用DJIMotor。
#define SHOOT_MOTOR_TIMEOUT_MS 20u      ///< 三电机反馈最大有效期，ms。
#define FRICTION_SPINUP_TIME_MS 300u    ///< 摩擦轮最短预热时间，ms。
#define FRICTION_READY_SPEED_RATIO 0.8f ///< 实测角速度达到目标的最低比例。
#define LOADER_JAM_CONFIRM_MS 100u      ///< 持续大电流确认窗口，ms。
#define LOADER_REVERSE_TIME_MS 100u     ///< 单次反转解堵窗口，ms。
#define HEAT_RESERVE 50u                // 原shoot_check的热量余量。

static DJIMotorInstance *friction_left_motor;           ///< 左摩擦轮，安装方向反转。
static DJIMotorInstance *friction_right_motor;          ///< 右摩擦轮，安装方向正常。
static DJIMotorInstance *loader_motor;                  ///< M2006拨弹电机。
static Subscriber_t *shoot_command_subscriber;          ///< 本轮发射请求。
static Subscriber_t *shoot_chassis_feedback_subscriber; ///< 底盘回传的真实裁判热量。
static Publisher_t *shoot_feedback_publisher;           ///< shoot_feed执行状态发布端。
static Shoot_Upload_Data_s shoot_feedback;              ///< 本轮三电机在线、轮速就绪、热量与拨弹状态。
static uint32_t friction_started_at_ms;                 ///< 摩擦轮本次启动时刻，HAL毫秒时间轴。
static uint32_t loader_jam_started_at_ms;               ///< 持续大电流检测起点，ms。
static uint32_t loader_reverse_started_at_ms;           ///< 本次反转解堵起点，ms。
static uint8_t friction_enabled;                        ///< 本轮之前是否已启动摩擦轮。
static uint8_t loader_jam_detected;                     ///< 是否处于连续大电流检测窗口。
static uint8_t loader_reversing;                        ///< 是否处于定时反转窗口。

/** @brief 停止三个发射电机并清空启动、堵转计时，下一次使能必须重新等待摩擦轮。 */
static void StopShooter(void)
{
    DJIMotorStop(friction_left_motor);
    DJIMotorStop(friction_right_motor);
    DJIMotorStop(loader_motor);
    friction_enabled = loader_jam_detected = loader_reversing = 0;
}

/** @brief 读取反馈是否存在且未超过20ms，在反馈临界区中调用。 */
static uint8_t IsShootMotorOnline(DJIMotorInstance *motor)
{
    return motor->feedback_seen && (uint32_t)(HAL_GetTick() - motor->last_rx_ms) <= SHOOT_MOTOR_TIMEOUT_MS;
}

void ShootInit(void)
{
    // 保留rm-sentry的0.85速度P环、相反安装方向及ID6/7，仅统一到云台CAN2。
    Motor_Init_Config_s friction_config = {
        .can_init_config =
            {
                .can_handle = SHOOT_CAN_HANDLE,
                .tx_id = SHOOT_FRICTION_RIGHT_ID,
            },
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp = 0.85f,
                        .Ki = 0,
                        .Kd = 0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 16000,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            },
        .motor_type = M3508,
    };
    friction_right_motor = DJIMotorInit(&friction_config);
    friction_config.can_init_config.tx_id = SHOOT_FRICTION_LEFT_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_left_motor = DJIMotorInit(&friction_config);

    // 原拨盘2.8速度P环，使用M2006协议输出上限；不复活原代码中未调通的位置环。
    Motor_Init_Config_s loader_config = {
        .can_init_config =
            {
                .can_handle = SHOOT_CAN_HANDLE,
                .tx_id = SHOOT_LOADER_ID,
            },
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp = 2.8f,
                        .Ki = 0,
                        .Kd = 0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 10000,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            },
        .motor_type = M2006,
    };
    loader_motor = DJIMotorInit(&loader_config);
    shoot_command_subscriber = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_chassis_feedback_subscriber = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    shoot_feedback_publisher = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    StopShooter();
}

/**
 * @brief 根据实测拨弹电流维护持续堵转与定时反转窗口。
 * @param loader_current_abs 电机反馈电流绝对值，单位沿用DJIMotor协议。
 * @param current_time_ms 本轮HAL毫秒时间戳，不能混用DWT时间轴。
 * @note 仅在允许拨弹时推进；停止拨弹时清空两个窗口，避免下次启动沿用旧计时。
 */
static void UpdateLoaderJamState(float loader_current_abs, uint32_t current_time_ms)
{
    // 原反转解堵逻辑增加100ms持续判定和100ms反转窗口，避免逐周期正反抖动。
    if (loader_current_abs > LOADER_JAM_CURRENT)
    {
        if (!loader_jam_detected)
        {
            loader_jam_started_at_ms = current_time_ms;
            loader_jam_detected = 1;
        }
        if (!loader_reversing && (uint32_t)(current_time_ms - loader_jam_started_at_ms) >= LOADER_JAM_CONFIRM_MS)
        {
            loader_reversing = 1;
            loader_reverse_started_at_ms = current_time_ms;
        }
    }
    else
    {
        loader_jam_detected = 0;
    }
    if (loader_reversing && (uint32_t)(current_time_ms - loader_reverse_started_at_ms) >= LOADER_REVERSE_TIME_MS)
    {
        loader_reversing = 0;
        loader_jam_detected = 0;
    }
}

/**
 * @brief 启动摩擦轮并判定本轮就绪；同时满足预热时长与两侧实测轮速才置位。
 * @param current_time_ms 本轮HAL毫秒时间戳。
 * @param left_speed_dps 左摩擦轮转子角速度绝对值，deg/s。
 * @param right_speed_dps 右摩擦轮转子角速度绝对值，deg/s。
 */
static void UpdateFrictionWheels(uint32_t current_time_ms, float left_speed_dps, float right_speed_dps)
{
    if (!friction_enabled)
    {
        friction_started_at_ms = current_time_ms;
        friction_enabled = 1;
    }
    DJIMotorSetRef(friction_left_motor, FRICTION_SPEED_DPS);
    DJIMotorSetRef(friction_right_motor, FRICTION_SPEED_DPS);
    DJIMotorEnable(friction_left_motor);
    DJIMotorEnable(friction_right_motor);
    // 速度达标且已启动300ms后才允许拨弹；这两项均为运行时实测反馈条件。
    shoot_feedback.friction_ready = (uint32_t)(current_time_ms - friction_started_at_ms) >= FRICTION_SPINUP_TIME_MS &&
                                    left_speed_dps >= FRICTION_SPEED_DPS * FRICTION_READY_SPEED_RATIO &&
                                    right_speed_dps >= FRICTION_SPEED_DPS * FRICTION_READY_SPEED_RATIO;
}

void ShootTask(void)
{
    Shoot_Ctrl_Cmd_s shoot_command = {0};
    Chassis_Upload_Data_s chassis_feedback = {0};
    uint8_t command_valid = SubGetMessage(shoot_command_subscriber, &shoot_command);
    uint8_t chassis_valid = SubGetMessage(shoot_chassis_feedback_subscriber, &chassis_feedback);
    uint32_t current_time_ms = HAL_GetTick();
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    shoot_feedback.motors_online = IsShootMotorOnline(friction_left_motor) &&
                                   IsShootMotorOnline(friction_right_motor) && IsShootMotorOnline(loader_motor);
    float left_speed_dps = fabsf(friction_left_motor->measure.speed_aps);
    float right_speed_dps = fabsf(friction_right_motor->measure.speed_aps);
    float loader_current_abs = fabsf(loader_motor->measure.real_current);
    __set_PRIMASK(interrupt_mask);

    shoot_feedback.friction_ready = 0;
    shoot_feedback.loader_running = 0;
    shoot_feedback.heat_allowed = chassis_valid && chassis_feedback.referee_online &&
                                  (unsigned)chassis_feedback.current_heat + HEAT_RESERVE < chassis_feedback.heat_limit;

    if (!command_valid || !shoot_feedback.motors_online || shoot_command.shoot_mode != SHOOT_ON ||
        shoot_command.friction_mode != FRICTION_ON)
    {
        StopShooter();
    }
    else
    {
        UpdateFrictionWheels(current_time_ms, left_speed_dps, right_speed_dps);
        if (shoot_command.load_mode == LOAD_BURSTFIRE && shoot_feedback.friction_ready && shoot_feedback.heat_allowed)
        {
            UpdateLoaderJamState(loader_current_abs, current_time_ms);
            DJIMotorSetRef(loader_motor, loader_reversing ? -LOADER_SPEED_DPS : LOADER_SPEED_DPS);
            DJIMotorEnable(loader_motor);
            shoot_feedback.loader_running = 1;
        }
        else
        {
            DJIMotorStop(loader_motor);
            loader_jam_detected = loader_reversing = 0;
        }
    }
    PubPushMessage(shoot_feedback_publisher, &shoot_feedback);
}
