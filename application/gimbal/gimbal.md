# 云台应用

## 总体说明

使用rm-sentry的DJIMotor、INS与消息中心，按旧舵轮映射注册
Yaw CAN1/GM6020 ID2、Pitch CAN2/GM6020 ID1。
不引入旧工程的CAN中断、RTOS或INS任务。

## 外部接口

`GimbalInit(void)` 在RobotInit中调用一次，初始化INS、电机和gimbal_cmd/gimbal_feed话题，
电机初始为停止状态。`GimbalTask(void)` 在RobotCMDTask后执行。
`get_IMU_data(void)` 返回固定地址的本轮INS快照，不转移所有权；
在线状态由gimbal_feed.online表示。

## 控制与私有状态

`imu_snapshot` 供OTHER_FEED使用，Yaw角度反馈为YawTotalAngle、速度反馈为Gyro[2]；
Pitch角度反馈为Pitch。
Yaw保留rm-sentry的角度+速度双环，Pitch保留其当前实际启用的角度环，
增益与目标反向关系见gimbal.c初始化项。

每轮检查INS有限数和反馈新鲜度、两个电机反馈20ms、CMD消息和目标有限数；
无力或无效时两轴停止、由MotorTask发送零电流。
gimbal_feedback_data.online表示原始传感器和电机有效，无力时仍可为1，供CMD判断可否解锁。
Yaw单圈角度保留float，避免将度数截断成uint16造成跟随误差。

## 调试边界

PID/INS方向取自rm-sentry，Yaw归中取自旧舵轮3938；
混合来源不等于已完成新硬件调参。先架空检查反馈符号、角度零位和机械行程。
CMD暂用±25度的Pitch目标限制，必须按实际机械行程核定。
