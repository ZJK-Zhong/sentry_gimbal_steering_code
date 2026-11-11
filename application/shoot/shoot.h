/**
 * @file shoot.h
 * @brief 双摩擦轮和拨弹轮的消息驱动速度控制接口
 */
#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief 在云台CAN2注册三个发射电机及目标/裁判反馈话题，初始零电流
 * @attention 由RobotInit创建任务前调用一次。
 */
void ShootInit(void);

/**
 * @brief 检查电机、摩擦轮准备和真实裁判热量，执行连发及定时反转解堵
 * @attention 由RobotTask每轮在CMD后调用，不可从其他任务并发调用。
 * @note 首帧未到、反馈过期或发射关闭时停止；无热量数据禁止拨弹。
 */
void ShootTask(void);
#endif
