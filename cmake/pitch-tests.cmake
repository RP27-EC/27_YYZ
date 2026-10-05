# No HAL/RTOS peripherals: production core and CAN driver with mocked HAL calls.
option(UP_BUILD_PITCH_TESTS "Build local ARM Pitch test image" OFF)
if(UP_BUILD_PITCH_TESTS)
    foreach(pitch_test_target IN ITEMS PitchTests PitchIntegralTests)
    add_executable(${pitch_test_target} EXCLUDE_FROM_ALL
        tests/pitch_tests.c Application/ModuleLayer/gimbal_pitch.c
        Application/ProtocolLayer/board_pitch_protocol.c Application/DriverLayer/drv_pitch.c
        Core/Src/system_stm32f4xx.c Core/Src/sysmem.c Core/Src/syscalls.c startup_stm32f407xx.s)
    target_compile_definitions(${pitch_test_target} PRIVATE STM32F407xx USE_HAL_DRIVER)
    target_compile_options(${pitch_test_target} PRIVATE -Wall -Wextra -Werror)
    target_include_directories(${pitch_test_target} PRIVATE Core/Inc Application/ModuleLayer
        Application/ConfigLayer Application/ProtocolLayer Application/DriverLayer
        Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/STM32F4xx_HAL_Driver/Inc)
    target_link_libraries(${pitch_test_target} PRIVATE m)
    target_link_options(${pitch_test_target} PRIVATE -Wl,-Map=${pitch_test_target}.map)
    endforeach()
    target_compile_definitions(PitchIntegralTests PRIVATE PITCH_ANGLE_KI=0.5f)
endif()
