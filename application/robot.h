/**
 * @file robot.h
 * @brief 双板云台主控的系统初始化和机器人周期任务入口
 */
#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief 初始化BSP、遥控/CAN命令、云台与发射应用，并创建基础任务
 * @attention main在外设初始化后、RTOS调度前调用一次。
 * @note 本板运行INS、电机、守护和机器人任务；裁判/UI在底盘板。
 */
void RobotInit(void);

/**
 * @brief 顺序执行RobotCMDTask、GimbalTask和ShootTask
 * @attention 仅由StartROBOTTASK周期调用，不允许其他任务并发调用。
 * @note 本函数不含延时；循环延时由robot_task.h负责。
 */
void RobotTask(void);
#endif
