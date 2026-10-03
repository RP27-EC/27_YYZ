# 27_YYZ_down — STM32H723 调试工程

当前默认构建已经切换为 STM32H723VGT6 / Cortex-M7 / GCC，使用原 down 目录。
ControlTask 现在同时运行 BMI088 IMU 更新和基础麦轮机械模式；LedTask 保留心跳。两任务运行在 FreeRTOS/CMSIS-RTOS v1 上。

## 麦轮遥控机械模式

已加入 DBUS 接收、FDCAN1 四轮反馈、麦轮解算和速度 PI。每次复位 `mec_output_enable` 默认 0，先确认遥控和电机反馈，再手动开启。
当前操作、Watch 变量、结构体树和文件职责见 [Application 底盘接入说明](docs/application-chassis.md)。整理前的实板验证记录见 [docs/mec-validation.md](docs/mec-validation.md)。

**当前程序已按英雄工程的 Application 职责分层，运行低速底盘机械模式和 IMU 更新，尚不能代替完整英雄整车控制程序。**
用户已确认遥控底盘可用；左上拨杆=s1、右上拨杆=s2，两者上1/中3/下2。
完整参考工程的模式映射、实时观察与修改变量的区别、当前 PI 参数差异见 [工程范围与操作说明](docs/project-scope-and-controls.md)。

## 修改记录约定

按用户要求，后续每轮文件修改都在 `docs/changes/` 新增一份 Markdown，逐文件说明新增、修改、删除的内容和原因，并写明验证情况。
本轮记录见 [2026-10-03 Application 整理说明](docs/changes/2026-10-03-application-layering.md)；上一轮底盘实现补记见 [2026-10-01 变更说明](docs/changes/2026-10-01-controls-and-scope.md)。

## 当前控制链

```text
UART5 → DBUS 解码 → rc_sensor
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
`mec_output_enable` 及 `mec_io.*` 保留。确认遥控有效、四轮在线且摇杆回中后，在暂停状态执行 `set variable mec_output_enable = 1`，继续运行并将右拨杆从停止档拨到中档；详见当前接入说明。

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
新的底盘验证见 `docs/mec-validation.md`。下载会更新板上对应 Flash 区域的固件；当前已包含电机输出任务，默认许可为关闭。

2026-10-03 整理后的正式固件编译通过；电脑执行 ARM 控制测试得到 211817 次检查、0 失败，新旧控制器 30000 周期输出一致。本轮没有下载或重新验证实板，整理前的成功遥控记录不能代替本次外设验证。
电脑测试的构建、依赖和执行方法见 [Application 底盘接入说明](docs/application-chassis.md)。

H7 CMSIS/HAL 与时基文件来自本地英雄下板参考工程，许可证保留在相应 Drivers 目录。
GNU H723 启动文件来自 ST 官方 cmsis-device-h7 v1.10.6，Core/LICENSE.txt 保留对应授权文本。
