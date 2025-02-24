# About

A collection of recipes for setting up various STM32 components using bare-metal, CMSIS, and HAL. 

These examples utilise the STM32L4 on the B-L4S5I-IOT01A Discovery board.

# Requirements
- ARM Toolchain, download it from ARM website here: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
- stm32-cmake,   CMakeList will clone it from github
- STM32Cube,     CMakeList will clone it from github


# How to Build each example
- Download and extract the ARM-M4 toolchain for your platform. 
- Set the toolchain path in var ARM_TOOLCHAIN_PATH within setupenv.sh
- source setupenv.sh
- Each folder is a stand-alone example with a CMakeList
- source create_configs.sh will create a debug and release configuration
- Select either debug or release, then make.

# Contents
### bareMetalBlink : A minimal bare-metal implementation of a "Blink LED" program.
- TBA

