# 云台CMD输入与输出编排

`RobotCMDInit()`初始化策略上下文、遥控、板间服务和本板话题；正常固件还初始化视觉采集。
`RobotCMDTask()`依次执行`ReadControlInputs`、`RobotControlUpdate`、`PublishControlOutputs`。

采集层每轮清零`RobotControlInput_s`，填充遥控/视觉/CAN/云台话题快照及有效标志。
分发层将云台和发射目标交给消息中心，将底盘目标交给BoardTransport，并刷新`gimbal_control`。
目标生成、解锁和巡查详见[控制策略](robot_control.md)，视觉有效期详见[视觉采集](vision_command.md)。

私有`robot_control_state`只属于CMD；各`*_publisher`、`*_subscriber`表示明确的话题方向。
`gimbal_control`保留原Watch字段和单位，仅接收观测副本；策略不会读取或被Watch写入驱动。
任务由RobotTask每轮在GimbalTask/ShootTask之前执行，末尾原5ms延时保留。
联调配置只采集遥控和板间反馈，不创建或伪造本板云台/发射反馈。
