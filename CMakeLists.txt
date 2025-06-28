cmake_minimum_required(VERSION 3.16)

set(CMAKE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(REPO_DIR "${CMAKE_SOURCE_DIR}/repos")
set(STM32_CMAKE "${REPO_DIR}/stm32-cmake")
set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE}/cmake/stm32_gcc.cmake")

# Detect ARM toolchain path - GitHub Actions installs to /opt/gcc-arm-none-eabi-*/bin
if(EXISTS "/opt/gcc-arm-none-eabi-12.2.rel1/bin")
    set(ARM_TOOLCHAIN_PATH "/opt/gcc-arm-none-eabi-12.2.rel1/bin")
elseif(EXISTS "/usr/local/bin")
    set(ARM_TOOLCHAIN_PATH "/usr/local/bin")
else()
    # Try to find it in PATH
    find_program(ARM_GCC_PATH arm-none-eabi-gcc)
    if(ARM_GCC_PATH)
        get_filename_component(ARM_TOOLCHAIN_PATH ${ARM_GCC_PATH} DIRECTORY)
    else()
        set(ARM_TOOLCHAIN_PATH "/usr/bin")
    endif()
endif()

set(ARM_CROSSTOOL_PREFIX "arm-none-eabi")
set(ARM_STM32_FAMILY "L4")

# Set the compiler path explicitly only if not already set
if(NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-gcc" CACHE STRING "C Compiler" FORCE)
endif()
if(NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-g++" CACHE STRING "C++ Compiler" FORCE)
endif()
if(NOT CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-gcc" CACHE STRING "ASM Compiler" FORCE)
endif()

# Define the project
project(stm32-recipes C CXX ASM)

# Set C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Debug and Release build options
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# Define STM32-related paths
set(STM32_CUBE_L4_PATH "${CMAKE_SOURCE_DIR}/repos/STM32CubeL4")
set(STM32_CMAKE "${CMAKE_SOURCE_DIR}/repos/stm32-cmake")

# Pass variables to subprojects
set(STM32_CUBE_L4_PATH ${STM32_CUBE_L4_PATH} CACHE INTERNAL "Path to STM32CubeL4")
set(STM32_CMAKE ${STM32_CMAKE} CACHE INTERNAL "Path to stm32-cmake")

# Top-level CMakeLists.txt
#find_package(CMSIS REQUIRED)
find_package(CMSIS COMPONENTS "${CMSIS_COMP_LIST}" REQUIRED)

# Include subprojects
add_subdirectory(bareMetalBlink)
add_subdirectory(bareMetalPWM)
add_subdirectory(bareMetalSysTick)

# Add more subprojects here as needed
# Example: add_subdirectory(anotherProject)
