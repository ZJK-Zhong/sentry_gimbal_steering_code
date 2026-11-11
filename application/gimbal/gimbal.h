/**
 * @file gimbal.h
 * @brief 云台两轴闭环的初始化、周期任务与INS快照接口
 */
#ifndef GIMBAL_H
#define GIMBAL_H
#include "ins_task.h"

/**
 * @brief 初始化INS、Yaw/Pitch电机和云台消息话题，初始输出零电流
 * @attention 由RobotInit在RobotCMDInit后、创建任务前调用一次。
 */
void GimbalInit(void);

/**
 * @brief 返回本轮云台INS快照的固定地址，单位deg和rad/s
 * @return attitude_t* 不转移所有权；在线状态以gimbal_feed.online为准。
 * @note 指针可长期持有，内容由GimbalTask周期更新，不得作为独立在线判据。
 */
attitude_t *get_IMU_data(void);

/**
 * @brief 读取目标/INS/电机反馈，设置两轴参考值或停止，并发布gimbal_feed
 * @attention 每轮由RobotTask在CMD后调用，电流由独立MotorTask统一发送。
 */
void GimbalTask(void);
#endif
