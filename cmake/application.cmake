# Application and CMSIS-DSP sources mirrored from MDK-ARM/My_C.uvprojx.
# Keep explicit sources: some Application files are not part of the down firmware.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Application/TaskLayer/community_task.c
    Application/TaskLayer/control_task.c
    Application/TaskLayer/led_task.c
    Application/TaskLayer/monitor_task.c
    Application/DeviceLayer/device.c
    Application/DeviceLayer/led.c
    Application/DeviceLayer/motor.c
    Application/DeviceLayer/Sensor/rc_sensor.c
    Application/DeviceLayer/Sensor/imu_sensor.c
    Application/DeviceLayer/imu_xrobot.c
    Application/DeviceLayer/Imu/BMI088Middleware.c
    Application/DeviceLayer/Imu/BMI088driver.c
    Application/DeviceLayer/Imu/bmi.c
    Application/DeviceLayer/Imu/bmi_EKF.c
    Application/HardwareLayer/RM_motor.c
    Application/HardwareLayer/HT_motor.c
    Application/HardwareLayer/DM_Motor.c
    Application/HardwareLayer/KT_motor.c
    Application/HardwareLayer/motor_def.c
    Application/ProtocolLayer/can_protocol.c
    Application/ProtocolLayer/rc_protocol.c
    Application/AlgorithmLayer/crc.c
    Application/AlgorithmLayer/PID.c
    Application/AlgorithmLayer/rp_math.c
    Application/AlgorithmLayer/ave_filter.c
    Application/AlgorithmLayer/kalman_filter.c
    Application/DriverLayer/drv_flash.c
    Application/DriverLayer/drv_uart.c
    Application/DriverLayer/drv_can.c
    Application/DriverLayer/drv_tim.c
    Application/DriverLayer/driver.c
    Application/DriverLayer/drv_tick.c
    Application/ConfigLayer/config_uart.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c
    Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c
    Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.c
    Drivers/CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c
    Drivers/CMSIS/DSP/Source/BayesFunctions/BayesFunctionsF16.c
    Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.c
    Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c
    Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c
    Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctionsF16.c
    Drivers/CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctionsF16.c
    Drivers/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c
    Drivers/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.c
    Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c
    Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c
    Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctionsF16.c
    Drivers/CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c
    Drivers/CMSIS/DSP/Source/SVMFunctions/SVMFunctionsF16.c
    Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
    Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctionsF16.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/AlgorithmLayer
    Application/ConfigLayer
    Application/DeviceLayer
    Application/DriverLayer
    Application/DeviceLayer/Imu
    Application/DeviceLayer/Sensor
    Application/HardwareLayer
    Application/ProtocolLayer
    Application/TaskLayer
    Application/UserLayer
    Drivers/CMSIS/DSP
    Drivers/CMSIS/DSP/Include
)

# CMSIS selects compiler intrinsics from the actual compiler predefined macros.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ARM_MATH_CM4)

