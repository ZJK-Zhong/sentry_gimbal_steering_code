# 发射应用

`ShootInit`注册左右摩擦轮、拨弹电机和shoot_cmd/chassis_feed/shoot_feed话题。
`ShootTask`采集本轮请求、真实裁判热量及三电机反馈，先判断全局执行前提，再调用两个私有步骤：
`UpdateFrictionWheels`维护预热与双侧实测轮速就绪；`UpdateLoaderJamState`维护持续大电流和反转窗口。
拨弹必须同时满足连发请求、摩擦轮就绪和热量允许；停止时清空堵转/反转状态。

三电机实例命名为`friction_left_motor`、`friction_right_motor`、`loader_motor`。
`*_started_at_ms`统一使用HAL时间轴；目标转速和实测`*_speed_dps`都是转子deg/s，
反馈电流沿用电机协议数值，不将其错误标成A。限幅、预热、热量和解堵配置均在文件开头说明。
通过话题通信，不读取CMD私有解锁状态；电流闭环和合帧发送由原MotorTask执行。
