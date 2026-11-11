# 云台主控 CMD

## 总体说明

接收机接USART3，保留rm-sentry的RemoteControl、message_center与CANComm，
按原左右拨杆组合选择整车模式。本板生成云台/发射目标，并通过云台CAN1向底盘CAN1发送命令。
本文件不直接发送电机电流，完整设备表见 [CAN分配](../../CAN_ALLOCATION.md)。

## 外部接口和类型

- RobotCMDInit：RobotInit中初始化遥控、USB视觉传输、CANComm和话题。
- RobotCMDTask：读取本轮反馈，解析模式、处理保护、生成目标并向两端分发。
- RobotControlMode_e：沿用rm-sentry的整车模式枚举，在robot_cmd.h定义。
- GimbalControlDebug_s / gimbal_control：selected_mode为拨杆请求，control_mode为实际执行模式；
  未解锁或保护生效时实际模式为SAFE，armed/vision_online可解释阻止执行的条件。

板间仍使用原26字节控制、28字节反馈；本次新增调试模式字段不经CAN发送，
不改变robot_def.h和board_link.h的线上布局。

## 拨杆组合

| 左拨杆 ＼ 右拨杆 | 下 | 中 | 上 |
| --- | --- | --- | --- |
| 下 | 无力模式 | 摩擦轮单开 | 遥控打弹 |
| 中 | 比赛模式（视觉） | 安全模式 | 安全模式 |
| 上 | 遥控移动/底盘跟随 | 小陀螺 | 云台打弹（视觉） |

左右都下既是无力，也是输入在线时的解锁位置。上电、遥控/CAN/本板云台反馈失效、
拨轮大于300急停后必须重新双下；右下不再单独表示零力。
两种安全组合停止底盘、云台、摩擦轮和拨弹轮，不残留上一模式的目标。

## 各模式行为

| 模式 | 底盘 | 云台 | 发射 |
| --- | --- | --- | --- |
| 移动 | 云台系手动平移、跟随Yaw | 手动 | 关闭 |
| 小陀螺 | 云台系手动平移、120度/秒自旋 | 手动 | 关闭 |
| 摩擦轮单开 | 手动平移，不跟随/不旋转 | 手动 | 仅摩擦轮 |
| 遥控打弹 | 手动平移，不跟随/不旋转 | 手动 | 请求连发 |
| 比赛 | 原视觉速度与旋转，导航坐标mode=1 | 视觉跟踪/巡查 | 原视觉请求 |
| 云台打弹 | 零力，按rm-sentry实际代码 | 视觉跟踪/巡查 | 原视觉请求 |
| 安全/无力 | 零力 | 零力 | 全部关闭 |

小陀螺、手动比例和Pitch限幅保留本舵轮已配置值；自动平移及旋转先按rm-sentry
-1400/+1400/2倍映射，再作2000mm/s、1500mm/s和πrad/s限幅。
云台Pitch统一限制±25度，机械行程与视觉安装补偿仍需实车核对。
ShootTask继续检查三个电机反馈、摩擦轮300ms/80%转速和真实裁判热量，模式请求不能绕过联锁。

## 视觉新鲜度和切换

ReadVisionCommand在短临界区复制原视觉模块的数据/时间戳，
使用与VisionCmdLastRxTimeMs相同的DWT时间轴，首帧未到、超过120ms、
非有限浮点数或非法开关字段时不采用。读取不刷新接收时间。
比赛/云台打弹视觉无效时，冻结云台目标，底盘停止，发射关闭；
只有通信在线但tracking=0才执行rm-sentry的-5～20度Pitch巡查和Yaw扫描。
视觉恢复后由当前模式重新生成命令。

每轮先清零输出，工作模式切换保留连续云台目标；
离开安全/无力时对齐当前反向INS反馈。模式变化立即发CAN，常态每10ms发送。

## 私有状态和调试

RobotModeChoose映射九种组合；UpdateRCInput只处理手动输入；
RunRobotControlMode分发各模式；UpdateVisionGimbal处理连续Yaw与巡查，
UpdateVisionShooter只生成请求。yaw_target/pitch_target保存目标，
patrol_direction保存巡查方向，active_last识别从无力恢复的边沿。

Watch建议：gimbal_control.selected_mode、control_mode、armed、rc_online、
chassis_online、gimbal_online、vision_online，以及三类cmd目标。

## 通信与控制的边界

CAN实例、超时快照和发送周期已移入[板间通信适配](../board_transport.md)。
CMD负责目标与策略；底盘执行层继续只订阅消息中心。

`control_state`保存私有整车状态，`gimbal_control`是任务结束后复制的Watch快照。
控制不读取该快照；对其写入不能改变解锁、目标或模式。

## 空上位板联调

没有可用云台时，使用默认关闭的`BENCH_TEST=1`构建；
接线、编译烧录、拨杆和Watch步骤见[空C板底盘联调](../../CHASSIS_BENCH_TEST.md)。
