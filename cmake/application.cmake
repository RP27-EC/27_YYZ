# Active application: preserve the original ControlTask and LedTask workflow.
# Layered H7 chassis application; inactive F4 templates remain excluded.
target_sources(My_C PRIVATE
    Application/ModuleLayer/shoot.c
    Application/ProtocolLayer/board_fric_protocol.c
    Application/DriverLayer/drv_pitch_link.c
    Application/ProtocolLayer/board_pitch_protocol.c
    Core/Src/chassis_board_h723.c
    Application/DriverLayer/drv_uart.c Application/DriverLayer/drv_can.c
    Application/DriverLayer/drv_status.c
    Application/ProtocolLayer/rc_protocol.c
    Application/ProtocolLayer/kt_yaw_protocol.c
    Application/DeviceLayer/Sensor/rc_sensor.c Application/DeviceLayer/motor.c
    Application/DeviceLayer/yaw_probe.c
    Application/HardwareLayer/RM_motor.c
    Application/ParameterLayer/chassis_param.c
    Application/ControlLayer/carctrl.c Application/ModuleLayer/chassis.c
    Application/ModuleLayer/gimbal.c Application/ModuleLayer/gyro_control.c
    Application/TaskLayer/control_task.c Application/TaskLayer/led_task.c
    Application/DeviceLayer/Sensor/imu_sensor.c
    Application/DeviceLayer/Imu/BMI088driver.c
    Application/DeviceLayer/Imu/BMI088Middleware.c
    Application/DeviceLayer/Imu/bmi.c
    Application/AlgorithmLayer/ave_filter.c
    Application/AlgorithmLayer/rp_math.c
    Application/AlgorithmLayer/PID.c
    Application/DriverLayer/drv_tick.c
)
target_include_directories(My_C PRIVATE
    Application/ControlLayer Application/ModuleLayer Application/HardwareLayer
    Application/DeviceLayer Application/ProtocolLayer Application/ParameterLayer
    Application/TaskLayer Application/DeviceLayer/Sensor Application/DeviceLayer/Imu
    Application/AlgorithmLayer Application/ConfigLayer Application/DriverLayer
    Drivers/CMSIS/DSP/Include
)
set(DSP ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source)
target_sources(My_C PRIVATE
    ${DSP}/FastMathFunctions/arm_sin_f32.c
    ${DSP}/FastMathFunctions/arm_cos_f32.c
    ${DSP}/FastMathFunctions/arm_atan2_f32.c
    ${DSP}/MatrixFunctions/arm_mat_init_f32.c
    ${DSP}/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP}/CommonTables/arm_common_tables.c
)
