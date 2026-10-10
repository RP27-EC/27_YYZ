# Independent diagnostic image; no RC, chassis, gimbal, motor or RTOS control sources.
add_executable(Can2Listen EXCLUDE_FROM_ALL
    diagnostics/can2_listen.c
    Core/Src/system_stm32h7xx.c Core/Src/runtime.c
    Core/Src/chassis_board_h723.c Core/Src/stm32h7xx_hal_timebase_tim.c
    Application/DriverLayer/drv_status.c
    Application/ProtocolLayer/board_pitch_protocol.c
    startup_stm32h723xx.s)
set_target_properties(Can2Listen PROPERTIES SUFFIX ".elf")
target_compile_definitions(Can2Listen PRIVATE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
target_compile_options(Can2Listen PRIVATE -Og -g3 -Wall -Wextra -Werror)
target_include_directories(Can2Listen PRIVATE
    Core/Inc Application/DriverLayer Application/ProtocolLayer
    Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy)
foreach(module hal hal_cortex hal_rcc hal_rcc_ex hal_pwr hal_pwr_ex hal_flash hal_flash_ex hal_gpio hal_tim hal_tim_ex hal_dma hal_dma_ex hal_fdcan)
    target_sources(Can2Listen PRIVATE ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_${module}.c)
endforeach()
target_link_options(Can2Listen PRIVATE
    "-T${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/Can2Listen.map" -Wl,--gc-sections -Wl,--print-memory-usage)
set_property(TARGET Can2Listen APPEND PROPERTY LINK_DEPENDS "${CMAKE_SOURCE_DIR}/STM32H723VG_FLASH.ld")
target_link_libraries(Can2Listen PRIVATE m)
