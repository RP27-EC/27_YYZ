option(DOWN_BUILD_SHOOT_TESTS "Build national shooter cross-board tests" ON)
if(DOWN_BUILD_SHOOT_TESTS)
    add_executable(ShootTests EXCLUDE_FROM_ALL tests/shoot_tests.c
        Application/ModuleLayer/shoot.c Application/ProtocolLayer/board_fric_protocol.c
        ../27_YYZ_up/Application/ModuleLayer/fric.c
        Core/Src/system_stm32h7xx.c Core/Src/runtime.c startup_stm32h723xx.s)
    set_target_properties(ShootTests PROPERTIES SUFFIX ".elf")
    target_compile_definitions(ShootTests PRIVATE STM32H723xx USE_PWR_LDO_SUPPLY)
    target_compile_options(ShootTests PRIVATE -mfpu=fpv5-sp-d16 -Og -g3 -Wall -Wextra -Werror)
    target_include_directories(ShootTests PRIVATE Core/Inc Application/ModuleLayer
        Application/ControlLayer Application/DeviceLayer/Sensor Application/ConfigLayer
        Application/ProtocolLayer ../27_YYZ_up/Application/ModuleLayer ../27_YYZ_up/Application/ConfigLayer
        Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32H7xx/Include)
    target_link_options(ShootTests PRIVATE -mfpu=fpv5-sp-d16 "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld" -Wl,--gc-sections)
    target_link_libraries(ShootTests PRIVATE m)
endif()
