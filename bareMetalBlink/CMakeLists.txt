cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE "$ENV{STM32_CMAKE}/cmake/stm32_gcc.cmake")

project(stm32-bareMetalBlink C CXX ASM)

# Set C and C++ standards (optional)
set(CMAKE_C_STANDARD 99)  # C99 standard for C files
set(CMAKE_CXX_STANDARD 17)  # C++17 standard for C++ files

# Configure here which STM32 target(s) to build
option(L4 "Compile L4 example" ON)

# Add Debug and Release build options
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# include
include_directories(${CMAKE_SOURCE_DIR}/inc)

# Specify paths to the source directories
set(C_DIR "src")
set(CPP_DIR "src")

# Use the find command to gather all .c and .cpp files
file(GLOB C_SOURCES "${C_DIR}/*.c")
file(GLOB CPP_SOURCES "${CPP_DIR}/*.cpp")

# Combine C and C++ sources
set(SRC_FILES ${C_SOURCES} ${CPP_SOURCES})

# CMSIS & HAL
set(CMSIS_INCLUDE_DIRS "${STM32_CUBE_L4_PATH}/Include")
set(CMSIS_SOURCES "${STM32_CUBE_L4_PATH}/Device/ST/STM32L4xx")
include_directories(${CMSIS_INCLUDE_DIRS})

find_package(CMSIS COMPONENTS "${CMSIS_COMP_LIST}" REQUIRED)

# STM32L4
if(L4)
    add_executable(stm32l4-bareMetalBlink ${SRC_FILES} )
    target_link_libraries(stm32l4-bareMetalBlink
        CMSIS::STM32::L4S5VI
        STM32::NoSys
    )
    stm32_print_size_of_target(stm32l4-bareMetalBlink)
endif()
