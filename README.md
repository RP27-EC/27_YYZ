# 27_YYZ_up — STM32F407 Pitch机械控制

上板使用GCC/CMake，负责达妙MIT Pitch电机反馈和速度单环/角度串级；下板通过CAN2转发遥控右摇杆上下或鼠标上下输入。本轮只更新上板，下板沿用现有版本。

详细烧录顺序和反馈见[Pitch接入与调试](docs/pitch-commissioning.md)。最新版本改为国赛机械模式：编码器角度外环、BMI088 IMU速度内环、逐周期积分及IMU角度重力补偿。机械参数为角度P=9/I=0.045、速度P=0.1/I=0，速度PI先限±5再加补偿。操作和对照见[国赛IMU版本](docs/pitch-national-imu.md)，逐文件记录见[本轮修改](docs/changes/2026-10-05-pitch-national-imu.md)。分环操作见[分环调参](docs/pitch-loop-tuning.md)。编译和电脑测试通过，实车稳定控制仍待验证。

- 上板调参：`Application/ConfigLayer/pitch_config.h`。
- 调试配置：`Up F407: download and debug`，固件`build/Debug/My_C.elf`。
- 首次保持右上拨杆停止档，开机静置至IMU校准完成、pitch_scope_imu_ready=1；手动确认抬头时IMU速度为正、压低为负，再释放输入执行停止档→中档。
- `gimbal_pitch_loop_mode=0`速度单环，右杆/鼠标上下直接给速度目标；当前默认`=1`角度串级，中档使能后目标以5°/s渐变到平射8.27833271°。在停止档切模式并重新启动。
- 用户要求对照试验：`PITCH_SPEED_GUARD_ENABLE=0`，默认关闭60°/s速度阈值退出，其他退出条件保留。恢复设为1后重新编译烧录；见[本轮记录](docs/changes/2026-10-05-pitch-speed-guard-off.md)。尖峰原因尚未确认。
- 源码风格见[AGENTS.md](AGENTS.md)，逐文件变更见[本轮修改记录](docs/changes/2026-10-04-pitch-control.md)。
