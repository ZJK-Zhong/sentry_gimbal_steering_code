/**
 * @file board_config.h
 * @brief 双板云台角色、CAN设备分配及原rm-sentry云台机械参数
 * @note 板间统一底盘CAN1↔云台CAN1；Yaw在云台CAN1/ID2。
 *       Pitch在云台CAN2/ID1，三个发射电机同接CAN2，电调ID均保持原配置。
 *       上位板两路设备整体对调，避免拨弹0x201/0x200与底盘驱动轮冲突。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
/* 默认运行完整云台主控；空C板联调请通过Make BENCH_TEST=1或CMake选项开启。
 * 此选项只改变本板应用和就绪条件，不改变板间协议、CAN分配或底盘保护。 */
#ifndef CHASSIS_BENCH_TEST
#define CHASSIS_BENCH_TEST 0
#endif
#if CHASSIS_BENCH_TEST != 0 && CHASSIS_BENCH_TEST != 1
#error "CHASSIS_BENCH_TEST must be 0 or 1"
#endif

#define GIMBAL_BOARD
#define BOARD_LINK_CAN_HANDLE (&hcan1)
#define GIMBAL_YAW_CAN_HANDLE (&hcan1)
#define GIMBAL_YAW_MOTOR_ID 2u
#define GIMBAL_PITCH_CAN_HANDLE (&hcan2)
#define GIMBAL_PITCH_MOTOR_ID 1u
#define SHOOT_CAN_HANDLE (&hcan2)
#define SHOOT_FRICTION_RIGHT_ID 6u
#define SHOOT_FRICTION_LEFT_ID 7u
#define SHOOT_LOADER_ID 1u
/* 旧舵轮yaw归中编码器3938；IMU方向及PID沿用rm-sentry，需实车重新核对安装方向。 */
#define YAW_CHASSIS_ALIGN_ECD 3938
#define YAW_ECD_GREATER_THAN_4096 0
#define PITCH_HORIZON_ECD 3412
#define PITCH_MAX_ANGLE 0
#define PITCH_MIN_ANGLE 0
#define ONE_BULLET_DELTA_ANGLE 36.0f
#define REDUCTION_RATIO_LOADER 36.0f
#define NUM_PER_CIRCLE 10
#define GYRO2GIMBAL_DIR_YAW 1
#define GYRO2GIMBAL_DIR_PITCH 1
#define GYRO2GIMBAL_DIR_ROLL 1
#endif
