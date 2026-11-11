/**
 * @file robot_control_config.h
 * @brief 整车目标生成参数，保留原有增益、限幅和安装补偿。
 * @note 每轮增量对应RobotTask末尾5ms延时；修改任务周期时必须重新核对。
 *       CAN端口与ID在board_config.h，机械归中编码器在robot_def.h。
 */
#ifndef ROBOT_CONTROL_CONFIG_H
#define ROBOT_CONTROL_CONFIG_H

#include "general_def.h"

#define MANUAL_SPIN_RADPS (120.0f * DEGREE_2_RAD) ///< 保留本舵轮已配置的120deg/s，不放大全向轮转速。
#define GIMBAL_PITCH_LIMIT_DEG 25.0f              ///< 沿用当前舵轮软件限幅，仍需核对实际机械行程。
#define PATROL_PITCH_MIN_DEG (-5.0f)              ///< 巡查Pitch下限，deg。
#define PATROL_PITCH_MAX_DEG 20.0f                ///< 巡查Pitch上限，deg。
#define PATROL_PITCH_STEP_DEG 0.15f               ///< 原rm-sentry每轮巡查增量，RobotTask延时5ms。
#define PATROL_YAW_STEP_DEG 0.4f                  ///< 每轮巡查Yaw增量，deg/轮。
#define VISION_YAW_COMPENSATION_DEG 0.65f         ///< 原rm-sentry视觉安装补偿，实车需核定。
#define VISION_PITCH_COMPENSATION_DEG 16.5f       ///< 视觉Pitch安装补偿，deg。

#define MANUAL_FORWARD_MMPS_PER_CHANNEL 6.0f     ///< 前后平移比例，mm/s/通道单位。
#define MANUAL_RIGHT_MMPS_PER_CHANNEL 2.0f       ///< 左右平移比例，mm/s/通道单位。
#define MANUAL_YAW_DEG_PER_CHANNEL 0.0008f       ///< Yaw每轮增量，deg/通道单位。
#define MANUAL_PITCH_DEG_PER_CHANNEL 0.002f      ///< Pitch每轮增量，deg/通道单位。
#define BENCH_YAW_DPS_PER_CHANNEL 0.2f           ///< 空板联调转向比例，deg/s/通道单位。
#define VISION_TRANSLATION_MMPS_PER_UNIT 1400.0f ///< 视觉平移比例，mm/s/协议单位。
#define VISION_SPIN_RADPS_PER_UNIT 2.0f          ///< 视觉转速比例，rad/s/协议单位。
#define REMOTE_STOP_DIAL_THRESHOLD 300           ///< 拨轮超过该值撤销解锁。
#define REMOTE_CHANNEL_DEADBAND 10               ///< DBUS通道中位死区。

#define YAW_ENCODER_DEG_PER_TICK 0.043945f ///< Yaw编码器每刻度角度；保留原电机模块舍入系数。

#endif
