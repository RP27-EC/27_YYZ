# 说明(修改中)

该文档为部分细节说明

## 陀螺仪GYRO模式下的底盘跟踪云台说明

底盘和云台的主要衔接是来自编码器的角度差

通过读取电机编码器两次角度的差值（还需经过归一化[−180°，180°)处理），作为跟踪函数的输入值x

            v = a × x + b × x²
            （具体函数：|v| = a × (4096 / 180) × |θ| + (F90 − a × 2048) × (|θ| / 90)²）

其中a为一次项系数，现实变现为小角度的跟踪力度，b为二次项系数，现实表现为大角度的跟踪力度，这里采用“一圈8192格”的刻度体系，因此a需乘以 (4096 /180)，当角度差达到90度时（180°折叠角度），函数输出最大。最后v作为轮子的旋转混合量，单位RPM

数据流动结构如下

    云台相对底盘角度 θ，单位：度
    ↓ Gyro_Follow()
    底盘旋转混合量 rotate，单位：转子RPM
    ↓ Chassis_Mix(front, right, rotate)
    四个轮子的目标转速，统一限幅
    ↓ RM_Motor_SpeedControl()
    轮速目标斜坡 → 与实际转速比较 → 速度PI
    ↓
    四个轮子的电流指令，原始CAN值
    ↓ CAN_Send()内部调用Motor_PackCurrent()
    CAN发送给电调 → 驱动电机

## 底盘解算简易说明

目前底盘轮子使用PI速度环控制

    左前目标 =  前进 + 右移 + 旋转;
    左后目标 =  前进 - 右移 + 旋转;
    右前目标 = -前进 + 右移 + 旋转;
    右后目标 = -前进 - 右移 + 旋转;

各个目标值在进入PI控制前会会进行统一限制（包含目标角度和加速度限制）。

    目标角度超出部分按比例统一缩小，保证运行模型不变
    本周期允许变化量 = acceleration_rpm_per_second × dt
    斜坡后目标 += 限幅( 原始轮速目标 − 斜坡后目标, ±本周期允许变化量)
## 修改记录约定

本轮Pitch上下板接入见[修改记录](docs/changes/2026-10-04-pitch-link.md)，实际烧录、观察及操作见[上板Pitch调试说明](../27_YYZ_up/docs/pitch-commissioning.md)。下板`pitch_link`观察上板状态，右杆上下/鼠标上下通过CAN2发送；需两板都烧录本轮固件。

按用户要求，后续每轮文件修改都在 `docs/changes/` 新增一份 Markdown，逐文件说明新增、修改、删除的内容和原因，并写明验证情况。
本轮记录见 [2026-10-03 Application 整理说明](docs/changes/2026-10-03-application-layering.md)；上一轮底盘实现补记见 [2026-10-01 变更说明](docs/changes/2026-10-01-controls-and-scope.md)。

脱离电脑运行配置与本轮修改见 [2026-10-03 独立运行说明](docs/changes/2026-10-03-standalone-rc.md)，烧录及参数操作见 [独立遥控与调参](docs/standalone-rc-and-parameters.md)。
键鼠接入逐文件记录见 [2026-10-03 WASD 修改说明](docs/changes/2026-10-03-keyboard-wasd.md)。
Yaw探测逐文件记录见 [2026-10-03 Yaw反馈探测修改说明](docs/changes/2026-10-03-yaw-feedback-probe.md)。后续源码风格见 [AGENTS.md](AGENTS.md)。
Yaw暂定零点与回绕处理见 [2026-10-03 Yaw角度修改说明](docs/changes/2026-10-03-yaw-angle-wrap.md)。
独立Yaw短时控制逐文件修改见 [2026-10-03 Yaw控制实现](docs/changes/2026-10-03-yaw-control.md)。
遥控Yaw角度外环、本地已调速度参数保留及观测字段见[2026-10-04角度串级修改记录](docs/changes/2026-10-04-yaw-remote-angle.md)。

## 当前控制链

```text
选手端 → USB遥控器 → 无线接收机 → UART5 → DBUS 解码 → rc_sensor（摇杆/键鼠）
FDCAN1 → 电机反馈 → rm_motor[4]
                         ↓
ControlTask → 设备快照 → car.work() → chassis.work() → CAN_Send()
                         模式检查     麦轮解算/速度环   0x200电流报文
```

模式在 `Application/ControlLayer/carctrl.c`，麦轮解算在 `Application/ModuleLayer/chassis.c`，速度 PI 在 `Application/HardwareLayer/RM_motor.c`。参数分别集中在 `ConfigLayer/chassis_config.h` 和 `ParameterLayer/chassis_param.c`。
`main.c` 调用 `Control_Init()` 初始化；`TaskLayer/control_task.c` 调度上述控制链。旧 `mec_mode.c`、`mec_io_h723.c` 已退出正式固件，旧控制核心只在 `tests/reference/` 保留作回归对照。

## 直接调试

1. VS Code 打开本目录 `E:/RP_hole_infantry/27_YYZ_down`。
2. Ctrl+Shift+B：执行 `H723: Build`。
3. 在“运行和调试”选择 `Down H723: download and debug`，按 F5。
4. 程序下载后停在 `Core/Src/main.c`；F5 继续，运行一会儿再暂停观察任务。
5. 可在 `Application/TaskLayer/control_task.c` 的 `imu_task_count++` 处设置断点。

Watch 建议添加：

- `debug_stage`：5 表示已经进入启动调度器阶段。
- `debug_core_clock_hz`：当前为 64000000（内部 HSI）。
- `debug_imu_init_code`：0 表示 BMI088 初始化成功。
- `debug_error_code`：0 表示没有进入本工程的 Error_Handler。
- `imu_task_count`、`imu_update_count`：任务执行与 IMU 更新次数。
- `imu_gyro_x`、`imu_gyro_y`、`imu_gyro_z`：陀螺仪读数。
- `imu_spi_error_count`：SPI 传输错误数。
- `led_task_count`、`led_task_heartbeat`：500 ms 心跳。

普通 Watch 在暂停时刷新；停在断点时任务不继续执行，计数不增长是正常的。

底盘 Watch 改用 `rc_sensor.info->frames`、`rc_sensor.info->s2.value`、`car.car_mode`、`car.block_reason`、`car.online_mask` 和 `rm_motor[CHAS_LF].info->speed`；旧 `mec_chassis.*` 已移除。
`mec_output_enable`及`mec_io.*`保留。当前底盘/Yaw固件默认许可为1，不需手动设置；确认通信有效、四轮及Yaw在线后，摇杆回中并将右拨杆从停止档拨到中档。若将CHASSIS_BOOT_OUTPUT_ENABLE改回0，可在机构静止、CPU暂停时用调试控制台赋值`mec_output_enable = 1`。Yaw参数修改在gimbal_config.h完成，重新编译烧录即可。

## 本次迁移范围

- 新的 H723 启动文件、H7 CMSIS/HAL、Flash/DTCM 链接脚本与 Cortex-M7/FPU 编译参数。
- TIM2 提供 HAL 毫秒时基；SysTick 由 FreeRTOS 使用；短延时使用 DWT 周期计数器。
- IMU 改用 SPI2：PB13 SCK、PC1 MOSI、PC2_C MISO、PC0 加速度片选、PC3 陀螺仪片选。
- IMU 初始化失败时返回可观察的错误状态，避免反复复位导致无法调试。
- 沿用现有 FreeRTOS v10.3.1 与 CMSIS-RTOS v1 两任务结构。
- H7 板的 LED 硬件接法不同，LedTask 当前只维护心跳变量，不再操作 F4 的 PH10。

本次接入涉及的 F4 电机、bxCAN、串口和遥控模板已归档在 `Legacy/F4/Application/`，对应 Application 文件现在是实际编译的 H7 底盘实现。其他未列入 CMake 的历史模板仍不参与编译。
当前使用 HSI 64 MHz 主频、轮询 SPI、UART5 中断接收；PLL1Q 单独提供 FDCAN 的 80 MHz 时钟，CAN1 使用经典 CAN 1 Mbit/s。尚未启用 DMA、D-cache，也不是英雄完整整车程序。
链接脚本将数据与栈放在 DTCM；将来启用普通 DMA 时须为其缓冲区选择 DMA 可访问的 SRAM。

## 文件与构建

- `CMakeLists.txt`、`cmake/h723-platform.cmake`：H7 平台与 RTOS 源文件。
- `cmake/application.cmake`：实际参加编译的业务源文件，使用明确列表。
- `cmake/gcc-arm-none-eabi.cmake`：Cortex-M7、fpv5-d16、hard-float。
- `build/H723_Debug/My_C.elf`：当前调试文件。旧 `build/Debug` 里的 F4 文件不会被新配置使用。
- `Legacy/F4`：原 Core、Keil、USB、CubeMX 与编译配置归档，供查阅。目录相对位置已变化，不应直接当成可构建工程。
- `Reference/DM-MC02.ioc`：英雄参考工程的完整 H7 板级配置，供核对引脚；不是当前程序的生成配置。
  当前 H7 初始化由 `board_h723.c` 手工维护，不能直接用该参考 ioc 覆盖生成。

原 F4 完整备份（排除 .git 与 build）：
`E:/RP_hole_infantry/backups/27_YYZ_down_F4_20261001-150505`

本机使用 STM32Cube 工具包 GCC 14.3.1+st.2、CMake 4.3.1+st.1、Ninja 1.13.2+st.1、J-Link 9.42.0+st.1。
新克隆后运行 `powershell -ExecutionPolicy Bypass -File scripts/setup-vscode.ps1` 可重建被忽略的本地 VS Code 配置。
构建可运行 `powershell -ExecutionPolicy Bypass -File scripts/build.ps1`。
H7 设备包使用本机已安装的 `STMicroelectronics.stm32h7xx_dfp.1.3.0`。

FreeRTOS 的 GCC/ARM_CM4F 目录同时支持 H723 所用的 Cortex-M7 r1p1，这是共享移植层的目录名称，并不表示仍在生成 M4 固件。
依据：https://github.com/FreeRTOS/FreeRTOS-Kernel/blob/main/portable/GCC/ARM_CM7/ReadMe.txt

## 实板验证

早期 IMU/心跳迁移记录见 `docs/H723-validation.md`；该记录早于底盘模块，不能用于证明新控制链已验证。
整理前的底盘验证见 `docs/mec-validation.md`。下载会更新板上对应Flash区域的固件；当前默认底盘/Yaw许可开启，启动仍由右拨杆手势和在线检查控制。运动中用J-Scope HSS采样；先右拨杆上/下停止并确认清零，再暂停/结束调试。脱离J-Link可以运行遥控，但本版没有无线变量回传。

2026-10-03 整理后的正式固件编译通过；电脑执行 ARM 控制测试得到 211817 次检查、0 失败，新旧控制器 30000 周期输出一致。该轮没有下载或重新验证实板，整理前的成功遥控记录不能代替外设验证。
电脑测试的构建、依赖和执行方法见 [Application 底盘接入说明](docs/application-chassis.md)。

本轮用户确认分层后的遥控控制实车成功。随后加入独立启动许可并编译通过，电脑测试211835次检查、0失败，含原30000周期回归及独立启动测试；该启动配置尚未下载或完成断开J-Link的重新上电验证。

本次 WASD 固件编译通过；键鼠测试558次检查、0失败，历史摇杆回归211835次检查、0失败。回归测试固定使用原低速参数，键鼠测试使用当前实际参数；未烧录本次键鼠版。测试镜像通过 `-DDOWN_BUILD_CONTROL_TESTS=ON` 显式启用，默认构建不依赖被忽略的本地 `tests/`。

2026-10-04遥控/键鼠Yaw版正式编译通过；Yaw/整车共享CAN电脑测试263474次检查、0失败，当前键鼠558次检查、0失败；旧旋转入口低速基线回归211835次检查/30000周期、0失败。基线镜像显式启用旧ch0旋转入口，正式固件关闭该入口；新映射由整车Yaw测试覆盖。本轮未自动烧录，模拟器通过不代表实车PID稳定。逐文件修改及验证见[本轮记录](docs/changes/2026-10-04-yaw-remote-control.md)。

用户随后确认速度环测试完成，当前源码速度Kp40、Ki2。本轮角度外环保留这些参数，增加遥控/鼠标累计目标、回中保持及任意当前位置启动；正式固件编译通过，Yaw电脑412058次检查、0失败，含原独立/整车速度回归及正反跨圈、保持纠偏、角度模式故障退出/恢复。未自动烧录，角度外环实车稳定性待验证。

随后用户已确认键鼠实车控制成功。新增Yaw探测版正式固件编译通过，Yaw探测549次电脑检查、键鼠558次检查、历史摇杆211835次检查/30000周期回归全部通过；本轮未烧录或确认实车Yaw反馈。

之后用户实车截图确认Yaw查询39次全部完成，收到32帧0x9C、7帧0x9A，在线且无发送错误；正前方向的另一组截图编码器为58768，观察到俯视逆时针递增与360°回绕。新角度处理版编译通过，Yaw测试1699次检查、0失败；尚未烧录或验证新增角度字段，尚未控制Yaw运动。

H7 CMSIS/HAL 与时基文件来自本地英雄下板参考工程，许可证保留在相应 Drivers 目录。
GNU H723 启动文件来自 ST 官方 cmsis-device-h7 v1.10.6，Core/LICENSE.txt 保留对应授权文本。
