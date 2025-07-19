cmake_minimum_required(VERSION 3.16)
project(bareMetalUART C CXX ASM)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/inc)

option(L4 "Compile L4 example" ON)

# Specify paths to the source directories
set(C_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Use the find command to gather all .c and .cpp files
file(GLOB C_SOURCES "${C_DIR}/*.c")
file(GLOB CPP_SOURCES "${CPP_DIR}/*.cpp")

# Combine C and C++ sources
set(SRC_FILES ${C_SOURCES} ${CPP_SOURCES})

# STM32L4
if(L4)
    add_executable(stm32l4-bareMetalUART ${SRC_FILES})
    target_link_libraries(stm32l4-bareMetalUART
        CMSIS::STM32::L4S5VI
        STM32::NoSys
    )
    stm32_print_size_of_target(stm32l4-bareMetalUART)
endif()
