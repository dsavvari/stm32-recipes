cmake_minimum_required(VERSION 3.16)

set(CMAKE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(REPO_DIR "${CMAKE_SOURCE_DIR}/repos")
set(STM32_CMAKE "${REPO_DIR}/stm32-cmake")
set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE}/cmake/stm32_gcc.cmake")
set(ARM_TOOLCHAIN_PATH "/usr/local/bin")
set(ARM_CROSSTOOL_PREFIX "arm-none-eabi")
set(ARM_STM32_FAMILY "L4")

# Set the compiler path explicitly
set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-gcc" CACHE STRING "C Compiler" FORCE)
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-g++" CACHE STRING "C++ Compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PATH}/${ARM_CROSSTOOL_PREFIX}-gcc" CACHE STRING "ASM Compiler" FORCE)

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

# Include subprojects
add_subdirectory(bareMetalBlink)

# Add more subprojects here as needed
# Example: add_subdirectory(anotherProject)