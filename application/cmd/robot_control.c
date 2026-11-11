/**
 * @file robot_control.c
 * @brief 输入快照到整车目标的策略，不执行外设读取和消息发送。
 */
#include "robot_control.h"
#include "robot_control_config.h"
#include "board_link.h"
#include "general_def.h"
#include <math.h>
#include <string.h>

/** @brief 按左右三段开关组合选择请求模式；联锁由更新接口单独处理。 */
RobotControlMode_e SelectRobotControlMode(uint8_t left_switch, uint8_t right_switch)
{
    switch (left_switch)
    {
    case RC_SW_DOWN:
        switch (right_switch)
        {
        case RC_SW_DOWN:
            return ROBOT_CONTROL_MODE_ZERO_FORCE;
        case RC_SW_MID:
            return ROBOT_CONTROL_MODE_FRICTION_ONLY;
        case RC_SW_UP:
            return ROBOT_CONTROL_MODE_FIRE;
        default:
            return ROBOT_CONTROL_MODE_SAFE;
        }
    case RC_SW_MID:
        return right_switch == RC_SW_DOWN ? ROBOT_CONTROL_MODE_COMPETITION : ROBOT_CONTROL_MODE_SAFE;
    case RC_SW_UP:
        switch (right_switch)
        {
        case RC_SW_DOWN:
            return ROBOT_CONTROL_MODE_MOVE;
        case RC_SW_MID:
            return ROBOT_CONTROL_MODE_SPIN;
        case RC_SW_UP:
            return ROBOT_CONTROL_MODE_GIMBAL_FIRE;
        default:
            return ROBOT_CONTROL_MODE_SAFE;
        }
    default:
        return ROBOT_CONTROL_MODE_SAFE;
    }
}

/** @brief 对目标作关于零的对称限幅，limit为非负幅值。 */
static float ClampControlValue(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

/** @brief 滤除DBUS中位死区，返回带符号通道值。 */
static int16_t ApplyRemoteDeadband(int16_t value)
{
    return value > REMOTE_CHANNEL_DEADBAND || value < -REMOTE_CHANNEL_DEADBAND ? value : 0;
}

#if !CHASSIS_BENCH_TEST
/** @brief 将角度差归一化到[-180,180]deg，保持连续Yaw目标。 */
static float WrapControlAngleDeg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    if (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/** @brief 更新手动平移和连续云台目标；通道比例及单位见配置头。 */
static void UpdateManualTargets(RobotControlState_s *state, const RC_ctrl_t *remote)
{
    state->chassis_cmd.vx = ClampControlValue(
        ApplyRemoteDeadband(remote->rc.rocker_r1) * MANUAL_FORWARD_MMPS_PER_CHANNEL, BOARD_MAX_VX_MMPS);
    state->chassis_cmd.vy =
        ClampControlValue(ApplyRemoteDeadband(remote->rc.rocker_r_) * MANUAL_RIGHT_MMPS_PER_CHANNEL, BOARD_MAX_VY_MMPS);
    state->yaw_target_deg += MANUAL_YAW_DEG_PER_CHANNEL * ApplyRemoteDeadband(remote->rc.rocker_l_);
    state->pitch_target_deg = ClampControlValue(state->pitch_target_deg + MANUAL_PITCH_DEG_PER_CHANNEL *
                                                                              ApplyRemoteDeadband(remote->rc.rocker_l1),
                                                GIMBAL_PITCH_LIMIT_DEG);
}

/** @brief 跟踪视觉角度；仅新鲜的无目标帧触发巡查，Pitch保持机械限幅。 */
static void UpdateVisionGimbalTargets(RobotControlState_s *state, const Vision_Recv_s *vision,
                                      const Gimbal_Upload_Data_s *feedback)
{
    if (vision->cmd.data.tracking.tracking)
    {
        float vision_yaw_deg =
            -(fmodf(vision->cmd.data.gimbal.yaw, 2.0f * PI) * RAD_2_DEGREE + VISION_YAW_COMPENSATION_DEG);
        float current_yaw_deg = -feedback->gimbal_imu_data.YawTotalAngle;
        state->yaw_target_deg = current_yaw_deg + WrapControlAngleDeg(vision_yaw_deg - current_yaw_deg);
        state->pitch_target_deg = -(vision->cmd.data.gimbal.pitch * RAD_2_DEGREE - VISION_PITCH_COMPENSATION_DEG);
        state->patrol_pitch_direction = 1;
    }
    else
    {
        state->pitch_target_deg += state->patrol_pitch_direction * PATROL_PITCH_STEP_DEG;
        if (state->pitch_target_deg >= PATROL_PITCH_MAX_DEG)
        {
            state->pitch_target_deg = PATROL_PITCH_MAX_DEG;
            state->patrol_pitch_direction = -1;
        }
        else if (state->pitch_target_deg <= PATROL_PITCH_MIN_DEG)
        {
            state->pitch_target_deg = PATROL_PITCH_MIN_DEG;
            state->patrol_pitch_direction = 1;
        }
        if (state->chassis_feedback.referee_online && state->chassis_feedback.current_hp != 0)
        {
            state->yaw_target_deg += PATROL_YAW_STEP_DEG;
        }
    }
    // 本舵轮保留±25deg输出范围，不照搬全向轮自动模式的±35deg范围。
    state->pitch_target_deg = ClampControlValue(state->pitch_target_deg, GIMBAL_PITCH_LIMIT_DEG);
}

/** @brief 转换视觉发射请求；实际轮速、预热和热量联锁交给Shoot应用。 */
static void UpdateVisionShootTargets(RobotControlState_s *state, const Vision_Recv_s *vision)
{
    state->shoot_cmd.shoot_mode = SHOOT_ON;
    state->shoot_cmd.friction_mode = vision->cmd.data.shoot.fric_on ? FRICTION_ON : FRICTION_OFF;
    state->shoot_cmd.load_mode = vision->cmd.data.shoot.fire ? LOAD_BURSTFIRE : LOAD_STOP;
}

/** @brief 按已解锁模式生成本轮目标，未赋值的动作保持安全默认值。 */
static void ApplyRobotControlMode(RobotControlState_s *state, RobotControlMode_e mode, const RC_ctrl_t *remote,
                                  const Vision_Recv_s *vision, const Gimbal_Upload_Data_s *feedback)
{
    Chassis_Ctrl_Cmd_s *chassis = &state->chassis_cmd;
    state->gimbal_cmd.gimbal_mode = GIMBAL_GYRO_MODE;
    chassis->mode = mode == ROBOT_CONTROL_MODE_COMPETITION ? 1 : 2;
    chassis->offset_angle =
        WrapControlAngleDeg(feedback->yaw_motor_single_round_angle - YAW_CHASSIS_ALIGN_ECD * YAW_ENCODER_DEG_PER_TICK);
    if (mode == ROBOT_CONTROL_MODE_COMPETITION || mode == ROBOT_CONTROL_MODE_GIMBAL_FIRE)
    {
        if (!state->vision_online)
        {
            // 原rm-sentry离线处理：底盘停止平移/旋转、关闭发射，云台保持当前目标。
            chassis->chassis_mode = CHASSIS_NO_FOLLOW;
            state->patrol_pitch_direction = 1;
        }
        else
        {
            UpdateVisionGimbalTargets(state, vision, feedback);
            UpdateVisionShootTargets(state, vision);
            if (mode == ROBOT_CONTROL_MODE_COMPETITION)
            {
                chassis->chassis_mode = CHASSIS_ROTATE;
                chassis->vx = ClampControlValue(-VISION_TRANSLATION_MMPS_PER_UNIT * vision->cmd.data.speed_vector.vx,
                                                BOARD_MAX_VX_MMPS);
                chassis->vy = ClampControlValue(VISION_TRANSLATION_MMPS_PER_UNIT * vision->cmd.data.speed_vector.vy,
                                                BOARD_MAX_VY_MMPS);
                chassis->chassis_spin_speed = ClampControlValue(
                    VISION_SPIN_RADPS_PER_UNIT * vision->cmd.data.speed_vector.wz, BOARD_MAX_WZ_RADPS);
            }
            // 云台打弹沿用原实际代码：底盘零力，只由视觉控制云台和发射。
        }
    }
    else
    {
        UpdateManualTargets(state, remote);
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
            state->shoot_cmd.shoot_mode = SHOOT_ON;
            state->shoot_cmd.friction_mode = FRICTION_ON;
            state->shoot_cmd.load_mode = mode == ROBOT_CONTROL_MODE_FIRE ? LOAD_BURSTFIRE : LOAD_STOP;
            break;
        default:
            break;
        }
    }
    state->gimbal_cmd.yaw = state->yaw_target_deg;
    state->gimbal_cmd.pitch = state->pitch_target_deg;
}

#else
/** @brief 空上位板只生成底盘目标；手动使用车体系，小陀螺使用入口朝向。 */
static void ApplyChassisBenchMode(RobotControlState_s *state, RobotControlMode_e mode, const RC_ctrl_t *remote)
{
    Chassis_Ctrl_Cmd_s *chassis = &state->chassis_cmd;
    chassis->vx = ClampControlValue(ApplyRemoteDeadband(remote->rc.rocker_r1) * MANUAL_FORWARD_MMPS_PER_CHANNEL,
                                    BOARD_MAX_VX_MMPS);
    chassis->vy =
        ClampControlValue(ApplyRemoteDeadband(remote->rc.rocker_r_) * MANUAL_RIGHT_MMPS_PER_CHANNEL, BOARD_MAX_VY_MMPS);
    if (mode == ROBOT_CONTROL_MODE_MOVE)
    {
        chassis->chassis_mode = CHASSIS_REMOTE;
        // ch2负号沿用已确认的实车转向；deg/s转为通信约定的rad/s。
        chassis->wz = ClampControlValue(
            -ApplyRemoteDeadband(remote->rc.rocker_l_) * BENCH_YAW_DPS_PER_CHANNEL * DEGREE_2_RAD, BOARD_MAX_WZ_RADPS);
    }
    else
    {
        chassis->chassis_mode = CHASSIS_ROTATE;
        chassis->mode = 3;
        chassis->chassis_spin_speed = MANUAL_SPIN_RADPS;
    }
}

#endif

void RobotControlInit(RobotControlState_s *state)
{
    memset(state, 0, sizeof(*state));
    state->patrol_pitch_direction = 1;
}

void RobotControlUpdate(RobotControlState_s *state, const RobotControlInput_s *input)
{
    RobotControlMode_e previous_mode = state->control_mode;
    const RC_ctrl_t *remote = &input->remote;

    // 每轮从安全默认值生成目标，禁止模式切换或失联后遗留上一轮动作。
    memset(&state->chassis_cmd, 0, sizeof(state->chassis_cmd));
    memset(&state->gimbal_cmd, 0, sizeof(state->gimbal_cmd));
    memset(&state->shoot_cmd, 0, sizeof(state->shoot_cmd));
    state->rc_online = input->remote_online;
    state->chassis_online = input->chassis_online;
    state->chassis_feedback = input->chassis_feedback;
    state->selected_mode = input->remote_online
                               ? SelectRobotControlMode(remote->rc.switch_left, remote->rc.switch_right)
                               : ROBOT_CONTROL_MODE_SAFE;
#if CHASSIS_BENCH_TEST
    // 空板不伪造云台在线；仅联调配置免去云台依赖。
    state->gimbal_online = 0;
    state->vision_online = 0;
    uint8_t inputs_ready = state->rc_online && state->chassis_online && remote->rc.dial <= REMOTE_STOP_DIAL_THRESHOLD;
#else
    state->gimbal_online = input->gimbal_online;
    state->vision_online = input->vision_online;
    uint8_t inputs_ready = state->rc_online && state->chassis_online && state->gimbal_online &&
                           remote->rc.dial <= REMOTE_STOP_DIAL_THRESHOLD;
#endif
    if (!inputs_ready)
    {
        state->armed = 0;
    }
    else if (state->selected_mode == ROBOT_CONTROL_MODE_ZERO_FORCE)
    {
        state->armed = 1;
    }
    state->control_mode = inputs_ready && state->armed ? state->selected_mode : ROBOT_CONTROL_MODE_SAFE;
#if CHASSIS_BENCH_TEST
    if (state->control_mode != ROBOT_CONTROL_MODE_MOVE && state->control_mode != ROBOT_CONTROL_MODE_SPIN &&
        state->control_mode != ROBOT_CONTROL_MODE_ZERO_FORCE)
    {
        state->control_mode = ROBOT_CONTROL_MODE_SAFE;
    }
#endif
    uint8_t active =
        state->control_mode != ROBOT_CONTROL_MODE_SAFE && state->control_mode != ROBOT_CONTROL_MODE_ZERO_FORCE;
    uint8_t mode_changed = previous_mode != state->control_mode;
#if CHASSIS_BENCH_TEST
    if (active)
    {
        ApplyChassisBenchMode(state, state->control_mode, remote);
    }
#else
    if (mode_changed)
    {
        state->patrol_pitch_direction = 1;
    }
    if (active)
    {
        if (!state->was_active)
        {
            // 沿用两轴目标反向约定；恢复控制时对齐当前INS，避免追赶停机前目标。
            state->yaw_target_deg = -input->gimbal_feedback.gimbal_imu_data.YawTotalAngle;
            state->pitch_target_deg = -input->gimbal_feedback.gimbal_imu_data.Pitch;
        }
        ApplyRobotControlMode(state, state->control_mode, remote, &input->vision, &input->gimbal_feedback);
    }
#endif
    state->command_send_immediately = mode_changed || (state->was_active && !active);
    state->was_active = active;
}
