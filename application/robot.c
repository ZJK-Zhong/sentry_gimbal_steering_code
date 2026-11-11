/**
 * @file robot.c
 * @brief 组织双板系统云台主控的初始化和应用调用顺序
 *
 * 初始化：BSPInit -> RobotCMDInit -> GimbalInit -> ShootInit -> OSTaskInit。
 * 正常配置保留原INS、电机、守护和机器人任务；裁判/UI由底盘板拥有。
 * 仅底盘联调配置只启动遥控/CAN策略与守护，不初始化本板云台、发射或INS。
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "referee_task.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"

// 提醒检查本板云台机械参数与旧舵轮硬件映射。
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

void RobotInit(void)
{
    // 初始化沿用全向轮框架；中断关闭期间只能使用 DWT 延时。
    __disable_irq();
    BSPInit();
    RobotCMDInit(); // 遥控/板间通信；正常配置另初始化USB视觉。
#if !CHASSIS_BENCH_TEST
    GimbalInit();   // 原rm-sentry云台闭环，硬件CAN和ID按旧舵轮配置。
    ShootInit();    // 两个摩擦轮和拨弹轮，均接云台CAN2。
#endif
    OSTaskInit();   // 联调只创建机器人和守护任务；正常配置另创建INS/电机任务。
    __enable_irq();
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t RobotTask_high_water;
#endif

void RobotTask(void)
{
    // 同一轮先发布目标，再由本板云台/发射应用取出执行；底盘目标通过CANComm下发。
    RobotCMDTask();
#if !CHASSIS_BENCH_TEST
    GimbalTask();
    ShootTask();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
    RobotTask_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
}
