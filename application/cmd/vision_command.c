/**
 * @file vision_command.c
 * @brief 原视觉传输的应用适配，集中处理首帧、DWT有效期和字段校验。
 */
#include "robot_def.h"
#if !CHASSIS_BENCH_TEST
#include "vision_command.h"
#include <math.h>
#include <string.h>

#define VISION_CMD_TIMEOUT_MS 120u           ///< 视觉命令最大有效期，DWT时间轴，ms。
static Vision_Recv_s *vision_receive_buffer; ///< 原视觉模块持有的接收缓冲，不能释放。
static uint8_t vision_first_frame_seen;      ///< 首帧到达前，零缓冲不代表有效命令。

void VisionCommandInit(void)
{
    vision_first_frame_seen = 0;
    vision_receive_buffer = VisionInit(&huart1);
}

uint8_t VisionCommandRead(Vision_Recv_s *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    if (vision_receive_buffer == NULL)
    {
        return 0;
    }
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if (VisionCmdHasNewFrame())
    {
        vision_first_frame_seen = 1;
    }
    uint32_t received_at_ms = VisionCmdLastRxTimeMs();
    uint32_t current_time_ms = (uint32_t)DWT_GetTimeline_ms();
    uint8_t valid = vision_first_frame_seen && (uint32_t)(current_time_ms - received_at_ms) <= VISION_CMD_TIMEOUT_MS;
    if (valid)
    {
        *snapshot = *vision_receive_buffer;
    }
    __set_PRIMASK(interrupt_mask);
    if (!valid)
    {
        return 0;
    }
    // tracking在原协议中为bool，按原始字节检查，拒绝非0/1的外部编码。
    uint8_t tracking;
    memcpy(&tracking, &snapshot->cmd.data.tracking.tracking, sizeof(tracking));
    valid = tracking <= 1 && snapshot->cmd.data.shoot.fire <= 1 && snapshot->cmd.data.shoot.fric_on <= 1 &&
            isfinite(snapshot->cmd.data.speed_vector.vx) && isfinite(snapshot->cmd.data.speed_vector.vy) &&
            isfinite(snapshot->cmd.data.speed_vector.wz) && isfinite(snapshot->cmd.data.gimbal.yaw) &&
            isfinite(snapshot->cmd.data.gimbal.pitch);
    if (!valid)
    {
        memset(snapshot, 0, sizeof(*snapshot));
    }
    return valid;
}
#endif
