/** @file robot_cmd.h
 * @brief 云台板遥控、底盘CANComm和本地云台/发射消息适配接口
 */
#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#include "robot_def.h"

/**
 * @brief 沿用rm-sentry的整车拨杆模式；本地调试状态，不改变板间CAN报文布局
 * @note 左下：右下无力/右中摩擦/右上打弹；左中：右下比赛，其余安全；
 *       左上：右下移动/右中小陀螺/右上云台视觉打弹。
 */
typedef enum
{
    ROBOT_CONTROL_MODE_SAFE = 0,
    ROBOT_CONTROL_MODE_COMPETITION,
    ROBOT_CONTROL_MODE_GIMBAL_FIRE,
    ROBOT_CONTROL_MODE_MOVE,
    ROBOT_CONTROL_MODE_SPIN,
    ROBOT_CONTROL_MODE_FRICTION_ONLY,
    ROBOT_CONTROL_MODE_FIRE,
    ROBOT_CONTROL_MODE_ZERO_FORCE,
} RobotControlMode_e;

/** @brief 云台CMD调试快照，单位与robot_def.h一致，不作为通信报文发送或控制输入。 */
typedef struct
{
    uint8_t bench_active; // 1为仅底盘联调固件；本字段不经CAN发送。
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
} GimbalControlDebug_s;
extern GimbalControlDebug_s gimbal_control;

/**
 * @brief 初始化云台USART3接收机、原USB视觉通信、消息中心及板间CANComm
 * @attention 由RobotInit在BSPInit之后、创建FreeRTOS任务前调用一次。
 * @note CHASSIS_BENCH_TEST=1时只初始化遥控、板间通信和底盘反馈话题。
 */
void RobotCMDInit(void);

/**
 * @brief 原子读取遥控与底盘状态，发布云台/发射目标并以100Hz发送底盘命令
 * @note 由RobotTask每5ms先于GimbalTask/ShootTask执行，按左右拨杆组合选择整车模式。
 *       左右均下为无力/解锁；遥控、CAN、云台反馈或拨轮急停后需回此组合。
 *       比赛和云台打弹依赖120ms内的新鲜视觉命令；超时保持云台、停止底盘和发射。
 *       仅底盘联调时不要求云台反馈，只开放双下无力、左上右下手动、左上右中小陀螺；
 *       手动按车体系并由ch2控制转向，小陀螺按底盘入口朝向平移；其余组合执行SAFE。
 */
void RobotCMDTask(void);
#endif
