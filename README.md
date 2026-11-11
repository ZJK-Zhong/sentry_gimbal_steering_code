# sentry_gimbal_steering_code

基于修改版 `E:/RM/rm-sentry` 框架的双板云台主控，接收机插 **云台USART3**，
通过 **云台CAN1 ↔ 底盘CAN1** 下发舵轮底盘命令。

本板运行CMD、INS、Yaw/Pitch闭环、两个摩擦轮和拨弹轮，原消息中心/电机模块统一发送电流。
按rm-sentry左右拨杆组合选择整车模式，左右均下为无力/解锁。

| 左拨杆 ＼ 右拨杆 | 下 | 中 | 上 |
| --- | --- | --- | --- |
| 下 | 无力模式 | 摩擦轮单开 | 遥控打弹 |
| 中 | 比赛模式（视觉） | 安全模式 | 安全模式 |
| 上 | 遥控移动/底盘跟随 | 小陀螺 | 云台打弹（视觉） |

无裁判热量、摩擦轮未就绪或任何必要反馈失效时禁止拨弹。

- [全部CAN设备及物理接线](CAN_ALLOCATION.md)：包含发射三电机和电容、裁判数据流。
- [调用链、遥控、调试和验证](README_STEERING.md)。
- [CMD接口](application/cmd/robot_cmd.md)、[云台应用](application/gimbal/gimbal.md)、[发射应用](application/shoot/shoot.md)。

构建：`mingw32-make -j24`，产物 `build/sentry_gimbal_steering_code.elf`。
比赛和云台打弹使用原USB视觉命令；首帧未到或超过120ms时停止底盘和发射、保持云台。
拨杆组合与模式执行已作软件测试，尚未烧录或连接实际视觉上位机验证。

模块职责、命名注释规范、控制/调试隔离及回归范围见[架构说明](ARCHITECTURE.md)。

## 空上位板联调

没有可用云台时，使用默认关闭的`BENCH_TEST=1`构建；
接线、编译烧录、拨杆和Watch步骤见[空C板底盘联调](CHASSIS_BENCH_TEST.md)。

阅读顺序：[输入/输出编排](application/cmd/robot_cmd.md) → [整车策略](application/cmd/robot_control.md)，
执行端见[云台](application/gimbal/gimbal.md)与[发射](application/shoot/shoot.md)。
