# 整车控制策略

## 文件和职责

`robot_control.h`声明模式、输入快照和独立状态；`robot_control.c`只执行策略；
`robot_control_config.h`集中保存手动比例、视觉补偿、巡查增量与限幅。
CMD负责串口/CAN/话题采集和发布；策略不直接读取这些接口，也不调用时钟。

## 接口与状态

`RobotControlInit(state)`清零状态并设置巡查初始方向。
`SelectRobotControlMode(left,right)`只映射拨杆请求，不处理解锁。
`RobotControlUpdate(state,input)`每轮清零三类目标、更新联锁、选择实际模式、生成目标和发送边沿。
`state`由调用方持有；不保存`input`指针，允许独立实例交错运行。

输入在线标志由采集层确认，不得用上轮数据延长有效期。正常固件要求遥控、板间反馈和本板云台就绪；
拨轮急停或任一前提失效撤销解锁，恢复后须双下重新解锁。
联调固件真实报告云台离线，只免去云台依赖，只允许无力、移动和小陀螺。

| 左拨杆 | 右下 | 右中 | 右上 |
| --- | --- | --- | --- |
| 下 | 无力/解锁 | 摩擦轮 | 发射 |
| 中 | 比赛 | SAFE | SAFE |
| 上 | 移动 | 小陀螺 | 视觉发射 |

手动与视觉辅助函数只更新传入上下文；连续Yaw/Pitch目标和巡查方向也存在该上下文中。
模式变化或刚停止时设置`command_send_immediately`，常态发送周期由BoardTransport管理。

## 调用示例

```c
RobotControlState_s state;
RobotControlInput_s input = {0};
RobotControlInit(&state);
/* 每轮由采集层填充input并确认各在线标志。 */
RobotControlUpdate(&state, &input);
/* 分发层发布state.chassis_cmd/gimbal_cmd/shoot_cmd。 */
```

Ozone继续看`gimbal_control`观测副本，不能以写Watch的方式驱动策略。
