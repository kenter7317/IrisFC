cmake_minimum_required(VERSION 3.20)

project(IrisFC C ASM)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/STM32F411CEUX_FLASH.ld)

file(GLOB_RECURSE SOURCES
    src/*.c
    src/*.s
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F411xE
)

target_include_directories(${PROJECT_NAME} PRIVATE
    src
    src/platform/Drivers/STM32F4xx_HAL_Driver/Inc
    src/platform/Drivers/CMSIS/Include
    src/platform/Drivers/CMSIS/Device/ST/STM32F4xx/Include
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -std=gnu11
    -Wall
)

target_link_options(${PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
)

add_custom_target(flash
    COMMAND st-flash --reset write $<TARGET_FILE:${PROJECT_NAME}> 0x08000000
    DEPENDS ${PROJECT_NAME}
)