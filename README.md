# STM32 Component Setup Recipes

## About
This repository provides a collection of examples for setting up various **STM32** components using **bare-metal**, **CMSIS**, and **HAL** frameworks. The examples are tailored for the **STM32L4** series, specifically using the **B-L4S5I-IOT01A Discovery Board**.

## Requirements
Before building the examples, make sure you have the following dependencies:

- **ARM Toolchain**: The ARM GCC toolchain is required for compiling the code. Download it from the [ARM Developer website](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).
- **STM32-CMake**: A CMake tool that simplifies STM32 project setup. The `CMakeLists.txt` will automatically clone it from GitHub.
- **STM32Cube**: Required for configuring STM32 peripherals and initialization. CMake will also clone this from GitHub as part of the setup.

## How to Build Each Example
Follow these steps to build the examples:

1. **Download and Extract the ARM Toolchain**: Choose the appropriate version of the ARM-M4 toolchain for your platform and extract it.
2. **Set the Toolchain Path**: Edit the `setupenv.sh` file and set the `ARM_TOOLCHAIN_PATH` variable to point to the toolchain directory.
3. **Source the Setup Script**: Run the following command to set up the environment variables:
    ```sh
    source setupenv.sh
    ```
4. **Build Configuration**: Each example is self-contained in its own folder with its own `CMakeLists.txt`. To configure build options, run:
    ```sh
    source create_configs.sh
    ```
    This will generate **Debug** and **Release** configurations.
5. **Select Build Type**: Choose either the **Debug** or **Release** configuration and run `make` to build:
    ```sh
    make <configuration>
    ```
    Replace `<configuration>` with either `debug` or `release`.

## Project Structure

### Example Folders
Each example is located in its own directory with a complete `CMakeLists.txt` file, making it easy to build and deploy independently.

#### `bareMetalBlink`
- A minimal **bare-metal** implementation of a "Blink LED" program. This example demonstrates how to set up basic GPIO for an LED blink with no external libraries (other than the necessary CMSIS).
  
#### `TBA`
- **To Be Added**: Future examples will be added to demonstrate other STM32L4 features such as UART, I2C, PWM, etc.

## License
This project is licensed under the **MIT License**. See the [LICENSE](./LICENSE) file for more details.
