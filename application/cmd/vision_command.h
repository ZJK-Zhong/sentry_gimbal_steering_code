/**
 * @file vision_command.h
 * @brief 将原视觉接收模块适配为带新鲜度检查的整车输入快照。
 */
#ifndef VISION_COMMAND_H
#define VISION_COMMAND_H

#include "master_process.h"

/**
 * @brief 初始化原视觉接收服务；在创建机器人任务前调用一次。
 * @note 传输方式沿用原USB VCP配置，不在本适配层解析线上报文。
 */
void VisionCommandInit(void);

/**
 * @brief 读取最近一条完整、有效且不超过120ms的视觉命令。
 * @param snapshot 非空输出；失败时清零，不能用于自动模式。
 * @return 1为有效，0为未收首帧、超时或字段非法。
 * @note 新鲜度使用原视觉模块的DWT时间轴；读取不会延长命令有效期。
 */
uint8_t VisionCommandRead(Vision_Recv_s *snapshot);

#endif
