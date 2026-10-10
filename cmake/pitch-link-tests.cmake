option(DOWN_BUILD_PITCH_LINK_TESTS "Build local ARM CAN2 Pitch input tests" OFF)
if(DOWN_BUILD_PITCH_LINK_TESTS)
    add_executable(PitchLinkTests EXCLUDE_FROM_ALL tests/pitch_link_tests.c
        Application/ModuleLayer/shoot.c Application/ProtocolLayer/board_fric_protocol.c
        Application/DriverLayer/drv_pitch_link.c Application/ProtocolLayer/board_pitch_protocol.c Application/ModuleLayer/gyro_control.c
        Core/Src/system_stm32h7xx.c Core/Src/runtime.c startup_stm32h723xx.s)
    set_target_properties(PitchLinkTests PROPERTIES SUFFIX ".elf")
    target_compile_definitions(PitchLinkTests PRIVATE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
    target_compile_options(PitchLinkTests PRIVATE -Og -g3 -Wall -Wextra -Werror)
    target_include_directories(PitchLinkTests PRIVATE Core/Inc Application/DriverLayer
        Application/ModuleLayer Application/ProtocolLayer Application/ConfigLayer Application/ControlLayer
        Application/DeviceLayer/Sensor Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy)
    target_link_options(PitchLinkTests PRIVATE "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld" -Wl,--gc-sections)
    target_link_libraries(PitchLinkTests PRIVATE m)
endif()
