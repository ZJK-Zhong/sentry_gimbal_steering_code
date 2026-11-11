/**
 * @file robot_cmd.c
 * @brief 按rm-sentry拨杆组合生成整车目标，使用原消息中心和CANComm分发
 * @note 遥控器接云台USART3；本板执行云台/发射，底盘命令经云台CAN1下发。
 *       拨杆只选择应用模式，本文件不直接发送电机电流。
 */
#include "robot_cmd.h"
#include "board_link.h"
#include "board_transport.h"
#include "remote_control.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include <math.h>
#include <string.h>

#define VISION_CMD_TIMEOUT_MS 120u // 与rm-sentry一致，按视觉模块的DWT接收时间判断。
#define MANUAL_SPIN_RADPS (120.0f * DEGREE_2_RAD) // 保留本舵轮已配置的120deg/s，不放大全向轮转速。
#define GIMBAL_PITCH_LIMIT_DEG 25.0f // 沿用当前舵轮软件限幅，仍需核对实际机械行程。
#define PATROL_PITCH_MIN_DEG (-5.0f)
#define PATROL_PITCH_MAX_DEG 20.0f
#define PATROL_PITCH_STEP_DEG 0.15f // 原rm-sentry每轮巡查增量，RobotTask延时5ms。
#define PATROL_YAW_STEP_DEG 0.4f
#define VISION_YAW_COMPENSATION_DEG 0.65f // 原rm-sentry视觉安装补偿，实车需核定。
#define VISION_PITCH_COMPENSATION_DEG 16.5f

static Publisher_t *chassis_feed_pub;
#if !CHASSIS_BENCH_TEST
static Vision_Recv_s *vision_recv_data;
static Publisher_t *gimbal_cmd_pub, *shoot_cmd_pub;
static Subscriber_t *gimbal_feed_sub;
static float yaw_target, pitch_target;
static int8_t patrol_direction = 1;
static uint8_t vision_seen;
#endif
static uint8_t active_last;
/* 私有整车控制状态；gimbal_control仅在任务结束后接收观测副本。 */
typedef struct
{
    uint8_t rc_online;
    uint8_t chassis_online;
    uint8_t gimbal_online;
    uint8_t armed;
    uint8_t vision_online;
    RobotControlMode_e selected_mode; // 拨杆请求；保护未解除时不等于执行模式。
    RobotControlMode_e control_mode;  // 本轮实际执行模式，未解锁/失联时为SAFE。
    Chassis_Ctrl_Cmd_s chassis_cmd;
    Gimbal_Ctrl_Cmd_s gimbal_cmd;
    Shoot_Ctrl_Cmd_s shoot_cmd;
    Chassis_Upload_Data_s chassis_feedback;
} RobotControlState_s;
static RobotControlState_s control_state;
GimbalControlDebug_s gimbal_control;

/** @brief 将本轮私有运行状态复制到观测快照；控制路径不读取快照。 */
static void PublishControlDebug(void)
{
    gimbal_control.bench_active = CHASSIS_BENCH_TEST;
    gimbal_control.rc_online = control_state.rc_online;
    gimbal_control.chassis_online = control_state.chassis_online;
    gimbal_control.gimbal_online = control_state.gimbal_online;
    gimbal_control.armed = control_state.armed;
    gimbal_control.vision_online = control_state.vision_online;
    gimbal_control.selected_mode = control_state.selected_mode;
    gimbal_control.control_mode = control_state.control_mode;
    gimbal_control.chassis_cmd = control_state.chassis_cmd;
    gimbal_control.gimbal_cmd = control_state.gimbal_cmd;
    gimbal_control.shoot_cmd = control_state.shoot_cmd;
    gimbal_control.chassis_feedback = control_state.chassis_feedback;
}

/**
 * @brief 按rm-sentry左右三段开关组合选择整车模式
 * @param left_switch 左侧开关原始编码，RC_SW_DOWN/MID/UP
 * @param right_switch 右侧开关原始编码，RC_SW_DOWN/MID/UP
 * @return RobotControlMode_e 未定义编码或组合返回SAFE，不沿用上一模式。
 */
static RobotControlMode_e RobotModeChoose(uint8_t left_switch, uint8_t right_switch)
{
    switch (left_switch)
    {
    case RC_SW_DOWN:
        switch (right_switch)
        {
        case RC_SW_DOWN: return ROBOT_CONTROL_MODE_ZERO_FORCE;
        case RC_SW_MID: return ROBOT_CONTROL_MODE_FRICTION_ONLY;
        case RC_SW_UP: return ROBOT_CONTROL_MODE_FIRE;
        default: return ROBOT_CONTROL_MODE_SAFE;
        }
    case RC_SW_MID:
        return right_switch == RC_SW_DOWN ? ROBOT_CONTROL_MODE_COMPETITION : ROBOT_CONTROL_MODE_SAFE;
    case RC_SW_UP:
        switch (right_switch)
        {
        case RC_SW_DOWN: return ROBOT_CONTROL_MODE_MOVE;
        case RC_SW_MID: return ROBOT_CONTROL_MODE_SPIN;
        case RC_SW_UP: return ROBOT_CONTROL_MODE_GIMBAL_FIRE;
        default: return ROBOT_CONTROL_MODE_SAFE;
        }
    default:
        return ROBOT_CONTROL_MODE_SAFE;
    }
}

/** @brief 对目标作对称限幅；limit是非负幅值，单位随输入。 */
static float Clamp(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

/** @brief DBUS中位死区，沿用舵轮独立底盘的±10通道单位。 */
static int16_t Deadband(int16_t value)
{
    return value > 10 || value < -10 ? value : 0;
}

#if !CHASSIS_BENCH_TEST
/** @brief 将角度差包回[-180,180]度，用于编码器偏角及视觉连续Yaw目标。 */
static float WrapAngle(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 复制原视觉模块最近的完整命令，并检查首帧、120ms新鲜度及数值
 * @param snapshot 非空视觉命令目标空间，任务私有副本
 * @return uint8_t 1:可用于自动模式；0:无首帧、超时或字段非法
 * @note 接收时间属于DWT时间轴，不能与HAL_GetTick混算；读取不延长有效期。
 */
static uint8_t ReadVisionCommand(Vision_Recv_s *snapshot)
{
    if (vision_recv_data == NULL) return 0;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (VisionCmdHasNewFrame()) vision_seen = 1;
    uint32_t received_at = VisionCmdLastRxTimeMs();
    uint32_t now = (uint32_t)DWT_GetTimeline_ms();
    uint8_t valid = vision_seen && (uint32_t)(now - received_at) <= VISION_CMD_TIMEOUT_MS;
    if (valid) *snapshot = *vision_recv_data;
    __set_PRIMASK(irq);
    if (!valid) return 0;
    // tracking在原协议中为bool，按原始字节检查，拒绝非0/1的外部编码。
    uint8_t tracking;
    memcpy(&tracking, &snapshot->cmd.data.tracking.tracking, sizeof(tracking));
    return tracking <= 1 && snapshot->cmd.data.shoot.fire <= 1 &&
        snapshot->cmd.data.shoot.fric_on <= 1 &&
        isfinite(snapshot->cmd.data.speed_vector.vx) &&
        isfinite(snapshot->cmd.data.speed_vector.vy) &&
        isfinite(snapshot->cmd.data.speed_vector.wz) &&
        isfinite(snapshot->cmd.data.gimbal.yaw) &&
        isfinite(snapshot->cmd.data.gimbal.pitch);
}

/**
 * @brief 更新手动平移与云台目标，不选择执行模式
 * @param rc 本轮有效遥控快照
 * @note 平移mm/s、云台deg；保留舵轮已验证的摇杆方向、比例和输出限幅。
 */
static void UpdateRCInput(const RC_ctrl_t *rc)
{
    control_state.chassis_cmd.vx = Clamp(Deadband(rc->rc.rocker_r1) * 6.0f, BOARD_MAX_VX_MMPS);
    control_state.chassis_cmd.vy = Clamp(Deadband(rc->rc.rocker_r_) * 2.0f, BOARD_MAX_VY_MMPS);
    yaw_target += 0.0008f * Deadband(rc->rc.rocker_l_);
    pitch_target = Clamp(pitch_target + 0.002f * Deadband(rc->rc.rocker_l1), GIMBAL_PITCH_LIMIT_DEG);
}

/**
 * @brief 沿用rm-sentry的视觉瞄准、连续Yaw处理及丢目标巡查
 * @param vision 已验证且未超时的视觉快照，角度输入rad
 * @param feedback 本板云台反馈，YawTotalAngle单位deg
 * @note 无视觉通信时不进入巡查；仅新鲜命令的tracking=0触发巡查。
 */
static void UpdateVisionGimbal(const Vision_Recv_s *vision, const Gimbal_Upload_Data_s *feedback)
{
    if (vision->cmd.data.tracking.tracking)
    {
        float target = -(fmodf(vision->cmd.data.gimbal.yaw, 2.0f * PI) * RAD_2_DEGREE + VISION_YAW_COMPENSATION_DEG);
        float reference = -feedback->gimbal_imu_data.YawTotalAngle;
        yaw_target = reference + WrapAngle(target - reference);
        pitch_target = -(vision->cmd.data.gimbal.pitch * RAD_2_DEGREE - VISION_PITCH_COMPENSATION_DEG);
        patrol_direction = 1;
    }
    else
    {
        pitch_target += patrol_direction * PATROL_PITCH_STEP_DEG;
        if (pitch_target >= PATROL_PITCH_MAX_DEG)
        { pitch_target = PATROL_PITCH_MAX_DEG; patrol_direction = -1; }
        else if (pitch_target <= PATROL_PITCH_MIN_DEG)
        { pitch_target = PATROL_PITCH_MIN_DEG; patrol_direction = 1; }
        if (control_state.chassis_feedback.referee_online && control_state.chassis_feedback.current_hp != 0)
            yaw_target += PATROL_YAW_STEP_DEG;
    }
    // 本舵轮保留±25deg输出范围，不照搬全向轮自动模式的±35deg范围。
    pitch_target = Clamp(pitch_target, GIMBAL_PITCH_LIMIT_DEG);
}

/** @brief 依据新鲜视觉请求设置发射目标；实测轮速、启动时间和热量仍由ShootTask联锁。 */
static void UpdateVisionShooter(const Vision_Recv_s *vision)
{
    control_state.shoot_cmd.shoot_mode = SHOOT_ON;
    control_state.shoot_cmd.friction_mode = vision->cmd.data.shoot.fric_on ? FRICTION_ON : FRICTION_OFF;
    control_state.shoot_cmd.load_mode = vision->cmd.data.shoot.fire ? LOAD_BURSTFIRE : LOAD_STOP;
}

/**
 * @brief 根据整车模式生成本轮目标，所有未设置的字段保持零力默认值
 * @param mode 已解锁且输入有效的非安全模式
 * @param rc 本轮遥控快照
 * @param vision 本轮视觉快照，仅vision_online为1时使用
 * @param feedback 本轮云台反馈
 */
static void RunRobotControlMode(RobotControlMode_e mode, const RC_ctrl_t *rc,
                               const Vision_Recv_s *vision, const Gimbal_Upload_Data_s *feedback)
{
    Chassis_Ctrl_Cmd_s *chassis = &control_state.chassis_cmd;
    control_state.gimbal_cmd.gimbal_mode = GIMBAL_GYRO_MODE;
    chassis->mode = mode == ROBOT_CONTROL_MODE_COMPETITION ? 1 : 2;
    chassis->offset_angle = WrapAngle(feedback->yaw_motor_single_round_angle -
        YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI);
    if (mode == ROBOT_CONTROL_MODE_COMPETITION || mode == ROBOT_CONTROL_MODE_GIMBAL_FIRE)
    {
        if (!control_state.vision_online)
        {
            // 原rm-sentry离线处理：底盘停止平移/旋转、关闭发射，云台保持当前目标。
            chassis->chassis_mode = CHASSIS_NO_FOLLOW;
            patrol_direction = 1;
        }
        else
        {
            UpdateVisionGimbal(vision, feedback);
            UpdateVisionShooter(vision);
            if (mode == ROBOT_CONTROL_MODE_COMPETITION)
            {
                chassis->chassis_mode = CHASSIS_ROTATE;
                chassis->vx = Clamp(-1400.0f * vision->cmd.data.speed_vector.vx, BOARD_MAX_VX_MMPS);
                chassis->vy = Clamp(1400.0f * vision->cmd.data.speed_vector.vy, BOARD_MAX_VY_MMPS);
                chassis->chassis_spin_speed = Clamp(2.0f * vision->cmd.data.speed_vector.wz, BOARD_MAX_WZ_RADPS);
            }
            // 云台打弹沿用原实际代码：底盘零力，只由视觉控制云台和发射。
        }
    }
    else
    {
        UpdateRCInput(rc);
        switch (mode)
        {
        case ROBOT_CONTROL_MODE_MOVE:
            chassis->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case ROBOT_CONTROL_MODE_SPIN:
            chassis->chassis_mode = CHASSIS_ROTATE;
            chassis->chassis_spin_speed = MANUAL_SPIN_RADPS;
            break;
        case ROBOT_CONTROL_MODE_FRICTION_ONLY:
        case ROBOT_CONTROL_MODE_FIRE:
            chassis->chassis_mode = CHASSIS_NO_FOLLOW;
            control_state.shoot_cmd.shoot_mode = SHOOT_ON;
            control_state.shoot_cmd.friction_mode = FRICTION_ON;
            control_state.shoot_cmd.load_mode = mode == ROBOT_CONTROL_MODE_FIRE ? LOAD_BURSTFIRE : LOAD_STOP;
            break;
        default:
            break;
        }
    }
    control_state.gimbal_cmd.yaw = yaw_target;
    control_state.gimbal_cmd.pitch = pitch_target;
}

#else
/**
 * @brief 空上位板联调时只生成底盘目标，不使用云台角度或视觉数据
 * @param mode 已通过解锁与输入检查的MOVE或SPIN
 * @param rc 本轮有效遥控快照
 * @note 手动使用车体系；小陀螺使用底盘捕获的入口朝向，底盘负责INS变换及转速爬升。
 */
static void RunChassisBenchMode(RobotControlMode_e mode, const RC_ctrl_t *rc)
{
    Chassis_Ctrl_Cmd_s *chassis = &control_state.chassis_cmd;
    chassis->vx = Clamp(Deadband(rc->rc.rocker_r1) * 6.0f, BOARD_MAX_VX_MMPS);
    chassis->vy = Clamp(Deadband(rc->rc.rocker_r_) * 2.0f, BOARD_MAX_VY_MMPS);
    if (mode == ROBOT_CONTROL_MODE_MOVE)
    {
        chassis->chassis_mode = CHASSIS_REMOTE;
        // ch2负号沿用已确认的实车转向；deg/s转为通信约定的rad/s。
        chassis->wz = Clamp(-Deadband(rc->rc.rocker_l_) * 0.2f * DEGREE_2_RAD, BOARD_MAX_WZ_RADPS);
    }
    else
    {
        chassis->chassis_mode = CHASSIS_ROTATE;
        chassis->mode = 3;
        chassis->chassis_spin_speed = MANUAL_SPIN_RADPS;
    }
}
#endif

void RobotCMDInit(void)
{
    RemoteControlInit(&huart3);
    BoardTransportInit();
    chassis_feed_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#if !CHASSIS_BENCH_TEST
    vision_recv_data = VisionInit(&huart1);
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
#endif
}

void RobotCMDTask(void)
{
    RC_ctrl_t rc = {0};
#if !CHASSIS_BENCH_TEST
    Gimbal_Upload_Data_s gimbal_feedback = {0};
    Vision_Recv_s vision = {0};
#endif
    RobotControlMode_e previous_mode = control_state.control_mode;
    // 每轮重置输出，禁止切换模式、消息缺失或保护生效时残留上一模式的动作。
    memset(&control_state.chassis_cmd, 0, sizeof(control_state.chassis_cmd));
    memset(&control_state.gimbal_cmd, 0, sizeof(control_state.gimbal_cmd));
    memset(&control_state.shoot_cmd, 0, sizeof(control_state.shoot_cmd));
    control_state.rc_online = RemoteControlSnapshot(&rc);
    control_state.selected_mode = control_state.rc_online ?
        RobotModeChoose(rc.rc.switch_left, rc.rc.switch_right) : ROBOT_CONTROL_MODE_SAFE;
    control_state.chassis_online = BoardTransportReadFeedback(
        &control_state.chassis_feedback);
#if CHASSIS_BENCH_TEST
    // 空板没有云台反馈，真实状态仍报告离线；仅本联调配置免去该依赖。
    control_state.gimbal_online = 0;
    control_state.vision_online = 0;
    uint8_t ready = control_state.rc_online && control_state.chassis_online && rc.rc.dial <= 300;
#else
    control_state.vision_online = ReadVisionCommand(&vision);
    control_state.gimbal_online = SubGetMessage(gimbal_feed_sub, &gimbal_feedback) &&
        gimbal_feedback.online && isfinite(gimbal_feedback.gimbal_imu_data.YawTotalAngle) &&
        isfinite(gimbal_feedback.gimbal_imu_data.Pitch) &&
        isfinite(gimbal_feedback.yaw_motor_single_round_angle);

    uint8_t ready = control_state.rc_online && control_state.chassis_online &&
        control_state.gimbal_online && rc.rc.dial <= 300;
#endif
    if (!ready) control_state.armed = 0;
    else if (control_state.selected_mode == ROBOT_CONTROL_MODE_ZERO_FORCE) control_state.armed = 1;
    control_state.control_mode = ready && control_state.armed ?
        control_state.selected_mode : ROBOT_CONTROL_MODE_SAFE;
#if CHASSIS_BENCH_TEST
    // 保留原拨杆请求用于排查；视觉、摩擦轮和发射组合在联调固件中实际执行SAFE。
    if (control_state.control_mode != ROBOT_CONTROL_MODE_MOVE &&
        control_state.control_mode != ROBOT_CONTROL_MODE_SPIN &&
        control_state.control_mode != ROBOT_CONTROL_MODE_ZERO_FORCE)
        control_state.control_mode = ROBOT_CONTROL_MODE_SAFE;
#endif
    uint8_t active = control_state.control_mode != ROBOT_CONTROL_MODE_SAFE &&
        control_state.control_mode != ROBOT_CONTROL_MODE_ZERO_FORCE;
    uint8_t mode_changed = previous_mode != control_state.control_mode;
#if CHASSIS_BENCH_TEST
    if (active) RunChassisBenchMode(control_state.control_mode, &rc);
#else
    if (mode_changed) patrol_direction = 1;
    if (active)
    {
        if (!active_last)
        {
            // rm-sentry两轴目标反向，离开无力/安全后先对齐当前INS，避免追赶旧姿态。
            yaw_target = -gimbal_feedback.gimbal_imu_data.YawTotalAngle;
            pitch_target = -gimbal_feedback.gimbal_imu_data.Pitch;
        }
        RunRobotControlMode(control_state.control_mode, &rc, &vision, &gimbal_feedback);
    }
#endif
    uint8_t stopped_now = active_last && !active;
    active_last = active;
    PubPushMessage(chassis_feed_pub, &control_state.chassis_feedback);
#if !CHASSIS_BENCH_TEST
    PubPushMessage(gimbal_cmd_pub, &control_state.gimbal_cmd);
    PubPushMessage(shoot_cmd_pub, &control_state.shoot_cmd);
#endif
    // 模式变化或保护停机立即提交，通信服务负责常态10ms节流。
    BoardTransportSendCommand(&control_state.chassis_cmd, mode_changed || stopped_now);
    PublishControlDebug();
}
