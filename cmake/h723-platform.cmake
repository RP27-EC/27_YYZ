set(HAL_DIR ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src)
foreach(module hal hal_cortex hal_rcc hal_rcc_ex hal_pwr hal_pwr_ex hal_flash hal_flash_ex hal_gpio hal_tim hal_tim_ex hal_spi hal_spi_ex hal_dma hal_dma_ex hal_uart hal_uart_ex hal_fdcan)
    target_sources(My_C PRIVATE ${HAL_DIR}/stm32h7xx_${module}.c)
endforeach()
set(RTOS ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
target_include_directories(My_C PRIVATE ${RTOS}/include ${RTOS}/CMSIS_RTOS ${RTOS}/portable/GCC/ARM_CM4F)
# FreeRTOS recommends this shared port for Cortex-M7 revisions newer than r0p1.
# H723 has a Cortex-M7 r1p1. This directory name does not select Cortex-M4 codegen.
foreach(file tasks queue list timers event_groups stream_buffer croutine)
    target_sources(My_C PRIVATE ${RTOS}/${file}.c)
endforeach()
target_sources(My_C PRIVATE ${RTOS}/CMSIS_RTOS/cmsis_os.c ${RTOS}/portable/MemMang/heap_4.c ${RTOS}/portable/GCC/ARM_CM4F/port.c)
