# 两轴云台应用

`GimbalInit`注册INS、Yaw/Pitch电机和原消息话题，保留当前PID与CAN映射。
`GimbalTask`先读取命令，再由`UpdateGimbalFeedback`采集INS/电机；`IsGimbalTargetValid`检查本轮执行条件。
目标及反馈就绪时只设参考和使能，否则两轴零电流；实际PID/CAN输出由原MotorTask执行。

私有`gimbal_yaw_motor`、`gimbal_pitch_motor`明确区分两轴；`imu_snapshot`地址固定，
其字段地址被电机OTHER_FEED持有，不可替换成任务局部对象。
`gimbal_feedback_data`通过gimbal_feed发布；Ozone可直接添加整个
`gimbal_feedback_data.gimbal_imu_data`查看本板INS，不需要逐个添加角度。
本板反馈不表示已将完整云台IMU发送到底盘。
