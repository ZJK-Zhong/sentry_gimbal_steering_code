# 视觉命令采集适配

`vision_command.h/.c`封装原视觉模块，统一处理首帧、120ms有效期和字段合法性。
`VisionCommandInit()`调用原`VisionInit(&huart1)`；实际USB/串口后端仍由原框架配置决定。
`VisionCommandRead(snapshot)`在短临界区复制完整命令，再验证tracking/fire/fric_on编码和速度/角度有限数。
返回0时输出清零；首帧前的零缓冲不视为在线。接收时刻与当前时刻都使用DWT时间轴。

私有`vision_receive_buffer`指向原模块持有的接收缓冲；`vision_first_frame_seen`记录首帧已到达。
调用方每轮读取快照并把返回值交给`RobotControlInput_s.vision_online`；读取不会刷新接收时间。
模块不决定比赛模式、巡查或开火动作，联调固件不初始化或调用本适配。
