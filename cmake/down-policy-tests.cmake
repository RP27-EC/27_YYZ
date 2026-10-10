# Current lower-board tolerance policy, production control and mocked peripherals.
option(DOWN_BUILD_POLICY_TESTS "Build lower-board fault policy tests" OFF)
if(DOWN_BUILD_POLICY_TESTS)
    add_executable(DownPolicyTests EXCLUDE_FROM_ALL tests/down_policy_tests.c
        Application/ModuleLayer/shoot.c Application/ProtocolLayer/board_fric_protocol.c
        Application/DriverLayer/drv_can.c Application/DriverLayer/drv_uart.c
        Application/DriverLayer/drv_status.c Application/ProtocolLayer/kt_yaw_protocol.c
        Application/DeviceLayer/yaw_probe.c Application/DeviceLayer/motor.c
        Application/ControlLayer/carctrl.c Application/ModuleLayer/gimbal.c
        Application/ModuleLayer/chassis.c Application/ModuleLayer/gyro_control.c
        Application/HardwareLayer/RM_motor.c Application/ParameterLayer/chassis_param.c
        Application/DeviceLayer/Sensor/rc_sensor.c Application/ProtocolLayer/rc_protocol.c
        Core/Src/system_stm32h7xx.c Core/Src/runtime.c startup_stm32h723xx.s)
    set_target_properties(DownPolicyTests PROPERTIES SUFFIX ".elf")
    target_compile_definitions(DownPolicyTests PRIVATE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
    target_compile_options(DownPolicyTests PRIVATE -mfpu=fpv5-sp-d16 -Og -g3 -Wall -Wextra -Werror)
    target_include_directories(DownPolicyTests PRIVATE Core/Inc Application/DriverLayer
        Application/ModuleLayer Application/ProtocolLayer Application/ConfigLayer
        Application/ControlLayer Application/DeviceLayer Application/DeviceLayer/Sensor
        Application/HardwareLayer Application/ParameterLayer Drivers/CMSIS/Include
        Application/TaskLayer Application/DeviceLayer/Imu Application/AlgorithmLayer
        Drivers/CMSIS/DSP/Include Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/STM32H7xx_HAL_Driver/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc/Legacy)
    target_link_options(DownPolicyTests PRIVATE -mfpu=fpv5-sp-d16
        "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld" -Wl,--gc-sections)
    target_link_libraries(DownPolicyTests PRIVATE m)
endif()
