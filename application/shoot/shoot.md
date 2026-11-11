# 发射应用

## 总体说明

沿用rm-sentry的DJIMotor速度环与消息中心，三个电机统一在云台CAN2：
右摩擦M3508 ID6、左摩擦M3508 ID7、拨弹M2006 ID1。
与Pitch GM6020 ID1共享总线时，反馈及电流ID不同，完整分配见根目录CAN_ALLOCATION.md。

## 外部接口和数据

`ShootInit(void)` 在RobotInit调用一次，注册三个电机及shoot_cmd/chassis_feed/shoot_feed话题。
`ShootTask(void)` 在CMD和GimbalTask之后周期调用。
`Shoot_Upload_Data_s` 含motors_online、friction_ready、heat_allowed、loader_running，
由本地shoot_feedback发布，不作为板间电机控制帧。

## 执行条件

两个摩擦轮保留0.85速度P环和相反方向，目标30000deg/s；
拨盘保留2.8速度P环、连发10000deg/s。
输出限制按M3508的16000、M2006的10000配置。

任一电机反馈未到或超过20ms、CMD缺失、发射关闭或摩擦关闭时，三个电机均停止。
摩擦轮启动至少300ms且两轮实测速度均达到目标80%，才置friction_ready。
请求连发还需底盘转来的referee_online和current_heat+50<heat_limit，
无裁判数据时仅可调试摩擦轮，不能拨弹。

拨弹电流超过原阈值13000持续100ms后，反转100ms解堵；
条件消失时停止拨弹并清空计时。当前仅启用速度连发和解堵，
保留的单发/三发枚举不表示位置环已经实现。

## 私有状态及调试

`friction_start_ms` 记录启动时间，`jam_start_ms/reverse_start_ms` 记录解堵阶段，
`StopShooter` 停止三个电机并清空启动/解堵状态。
Watch：shoot_feedback、friction_l/r->measure、loader->measure。
零力由MotorStop输出零电流，不是速度零闭环制动。
