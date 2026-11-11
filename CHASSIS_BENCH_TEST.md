# 空 C 板与底盘联调

## 用途与边界

仅有舵轮底盘和一块空 C 板时，空板可以作为上位主控：USART3接收遥控器，CAN1下发底盘命令。
不需要上位板的Yaw/Pitch、发射电机、INS或视觉上位机；底盘仍使用完整八电机、INS和原保护链路。
配置默认关闭，正常构建仍要求云台就绪。此配置不改变协议、CAN端口或电机ID。

## 接线与固件

1. 底盘板烧录同级`sentry_chassis_steering_code`正常构建的固件，不能使用默认本地遥控的`Steering-drive-chassis`替代。
2. 空C板烧录本工程的联调固件。遥控器接收机从底盘移到空板USART3/DBUS接口。
3. 空板CAN1接底盘CAN1：H接H、L接L，公共地，双方正确供电；两端终端电阻按实际总线核对。
   总线1Mbit/s，底盘CAN1上的驱动电机保持原接法；底盘CAN2仍连接舵向电机和电容。

本工程目录下使用原MSYS2环境运行：

```sh
mingw32-make -j24 BENCH_TEST=1
mingw32-make BENCH_TEST=1 download_dap
```

联调产物是`build-bench/sentry_gimbal_steering_code_bench.elf`（同时生成hex/bin），
与正常`build/sentry_gimbal_steering_code.elf`分目录存放。使用J-Link时将download_dap替换为download_jlink。
恢复完整云台版：`mingw32-make -j24`后`mingw32-make download_dap`。
VS Code任务新增`build chassis bench`、`download chassis bench dap/jlink`；
调试选择`Chassis bench DAPlink`，确保符号与联调固件一致。

CMake等效选项（另建目录）：

```sh
cmake -S . -B build-bench-cmake -G Ninja -DCHASSIS_BENCH_TEST=ON
cmake --build build-bench-cmake
```

CMake保留原目标名，联调ELF是`build-bench-cmake/sentry_gimbal_steering_code.elf`。

## 拨杆与坐标

上电保持底盘静止以完成本板INS校准，摇杆回中；先将左右开关都拨到下档解锁。
首次试转先架空底盘，确认轮向后再落地。

| 左开关 | 右开关 | 实际行为 |
| --- | --- | --- |
| 下 | 下 | 无力；遥控和板间反馈有效、拨轮≤300时解锁 |
| 上 | 下 | 手动：右摇杆平移，左摇杆水平控制旋转 |
| 上 | 中 | 小陀螺：120deg/s目标，由底盘限加速度；右摇杆平移 |
| 其余组合 | 任意 | 安全停机，不运行视觉、摩擦轮或发射 |

表中“其余”排除前三种组合。组合沿用双板rm-sentry布局；不同于单板右下/中/上的布局。
手动平移按当前车体系：X向前、Y向右；左摇杆旋转方向沿用已验证的负ch2映射。
小陀螺平移按进入该模式时的底盘朝向：进入时前方为+X、右方为+Y，底盘INS补偿后续转动。
该参考不是坏云台的朝向，也不表示具备位置闭环或消除INS漂移。

遥控失联、底盘回传超时或拨轮>300会撤销解锁；恢复后必须双下重新解锁。
空板仍以10ms周期发送26字节命令，底盘每20ms回传28字节反馈；上位反馈100ms、底盘命令50ms超时。
底盘八电机20ms反馈保护与小陀螺INS要求保持有效，无裁判和电容时使用原功率回退策略。
上位`chassis_online`只表示反馈通信有效；是否具备电机/INS条件请分别看回传标志与底盘实际状态。

## 调试入口

| 板子 | Watch字段 | 正常预期 |
| --- | --- | --- |
| 空C板 | gimbal_control.bench_active | 1，否则烧录/加载了正常云台版 |
| 空C板 | gimbal_control.rc_online / chassis_online | 接收机、CAN通后均为1 |
| 空C板 | gimbal_control.gimbal_online / vision_online | 均为0，此配置不伪造就绪 |
| 空C板 | gimbal_control.armed | 双下解锁后为1 |
| 空C板 | gimbal_control.selected_mode / control_mode | 拨杆请求/实际执行；受禁模式实际SAFE |
| 空C板 | gimbal_control.chassis_cmd | 手动CHASSIS_REMOTE/mode0；小陀螺CHASSIS_ROTATE/mode3 |
| 空C板 | gimbal_control.chassis_feedback.motors_online / ins_online | 反映底盘真实就绪情况 |
| 两板 | board_link_debug.rx_online / last_rx_ms | 在线为1；接收时间持续更新 |
| 底盘 | swerve_debug.mode / motors_online / ins_online | 实际执行模式、八电机/本板INS就绪 |
| 底盘 | swerve_debug.wheel_actual_rpm / steering_actual_deg | 四驱动与四舵向反馈 |

快照只用于观察，不能靠写Watch绕过保护。恢复云台硬件后烧录正常版本并按正常联锁测试。

## 实现与验证

board_config.h定义默认关闭开关；Make/CMake传入编译选项。
robot.c与robot_task.h在联调配置下只初始化CMD并运行机器人/守护任务，不启动本板电机、INS和视觉。
CMD继续使用RemoteControl、BoardTransport与原CANComm，不修改驱动或线上消息布局。
tests/bench.c不提供云台/发射反馈，使用真实DBUS解码、消息中心及CANComm分包，
检查九组合、坐标/方向、失联停机与重新解锁；tests/run.ps1同时运行完整云台与联调回归。
这些是软件验证，不能替代接线、实际总线和实车测试。
