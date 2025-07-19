cmake_minimum_required(VERSION 3.16)

# Project name
set(PROJECT_NAME cortexPowerModes)

# Create the executable target
add_executable(stm32l4-${PROJECT_NAME}
    src/main.c
)

# Set target properties
set_target_properties(stm32l4-${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Link libraries
target_link_libraries(stm32l4-${PROJECT_NAME}
    CMSIS::STM32::L4S5VI
    STM32::NoSys
)

# Print memory usage
stm32_print_size_of_target(stm32l4-${PROJECT_NAME})
