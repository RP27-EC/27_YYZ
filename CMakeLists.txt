cmake_minimum_required(VERSION 3.22)
project(My_C LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(My_C
    Core/Src/main.c Core/Src/runtime.c Core/Src/system_stm32h7xx.c
    Core/Src/board_h723.c Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/freertos.c startup_stm32h723xx.s
)
set_target_properties(My_C PROPERTIES SUFFIX ".elf")
target_compile_definitions(My_C PRIVATE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY ARM_MATH_CM7)
target_compile_options(My_C PRIVATE -Og -g3 -Wall -Wextra)
target_include_directories(My_C PRIVATE
    Core/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
)
include(cmake/h723-platform.cmake)
include(cmake/application.cmake)
target_link_options(My_C PRIVATE
    "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/My_C.map" -Wl,--gc-sections -Wl,--print-memory-usage)
set_property(TARGET My_C APPEND PROPERTY LINK_DEPENDS "${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld")
target_link_libraries(My_C PRIVATE m)
add_custom_command(TARGET My_C POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:My_C> ${CMAKE_BINARY_DIR}/My_C.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:My_C> ${CMAKE_BINARY_DIR}/My_C.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:My_C> VERBATIM)

# Separate MCU test image: no HAL, RTOS, receiver or motor peripheral is started.
# Build explicitly with: cmake --build --preset Debug --target MecTests
add_executable(MecTests EXCLUDE_FROM_ALL
    tests/mec_tests.c tests/reference/mec_mode.c
    Application/ProtocolLayer/rc_protocol.c Application/DeviceLayer/Sensor/rc_sensor.c
    Application/DeviceLayer/motor.c Application/HardwareLayer/RM_motor.c
    Application/ParameterLayer/chassis_param.c Application/ControlLayer/carctrl.c
    Application/ModuleLayer/chassis.c
    Core/Src/system_stm32h7xx.c Core/Src/runtime.c startup_stm32h723xx.s)
set_target_properties(MecTests PROPERTIES SUFFIX ".elf")
target_compile_definitions(MecTests PRIVATE STM32H723xx USE_PWR_LDO_SUPPLY)
target_compile_options(MecTests PRIVATE -Og -g3 -Wall -Wextra -Werror)
target_include_directories(MecTests PRIVATE tests/reference
    Application/ModuleLayer Application/ControlLayer Application/ProtocolLayer
    Application/DeviceLayer Application/DeviceLayer/Sensor Application/HardwareLayer
    Application/ConfigLayer Application/ParameterLayer
    Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32H7xx/Include)
target_link_options(MecTests PRIVATE "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld" -Wl,--gc-sections)
target_link_libraries(MecTests PRIVATE m)
