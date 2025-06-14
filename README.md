# STM32 Component Setup Recipes

## About
This repository provides a collection of examples for setting up various **STM32** components using **bare-metal**, **CMSIS**, and **HAL** frameworks. The examples are tailored for the **STM32L4** series, specifically using the **B-L4S5I-IOT01A Discovery Board**.

## Requirements
Before building the examples, ensure you have the following dependencies installed:

- **ARM Toolchain**: [Download the ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).
- **STM32-CMake**: A CMake tool that simplifies STM32 project setup. The `CMakeLists.txt` will automatically clone it from GitHub if not already present.
- **STM32Cube**: Required for configuring STM32 peripherals and initialization. CMake will also clone this from GitHub as part of the setup.

## How to Build the Examples

Follow these steps to build and run the examples:

1. **Set the Toolchain Path**:
   Ensure the ARM toolchain path is correctly set in the `CMakeLists.txt` file:
   ```cmake
   set(ARM_TOOLCHAIN_PATH "/path/to/arm-toolchain")
   ```
2. **Source the Thirdparty libraries**: Run the following command to set up the environment variables:
    ```sh
    source thirdparty.sh
    ```
3. **CMake Configuration**: Each example is self-contained in its own folder with its own `CMakeLists.txt`. To configure build options, run:
    ```sh
    mkdir build & cd build 
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    or
    mkdir rel & cd rel
    cmake -DCMAKE_BUILD_TYPE=Release ..
    ```
    This will generate **Debug** and **Release** configurations.
4. **Select Build Type**: Choose either the **Debug** or **Release** configuration and run `make` to build:
    ```sh
    make <target>
    ```
## Project Structure

### Example Folders
Each example is located in its own directory with a complete `CMakeLists.txt` file, making it easy to build and deploy independently.

#### `bareMetalBlink`
- A minimal **bare-metal** implementation of a "Blink LED" program. This example demonstrates how to set up basic GPIO for an LED blink with no external libraries (other than the necessary CMSIS).
  
#### `bareMetalPWM`
- A bare-metal example demonstrating **PWM (Pulse Width Modulation)** signal generation on STM32L4S5VI (B-L4S5I-IOT01A). This example configures TIM2 to output a PWM signal on PA0 (ARD-D1), and allows the user to cycle through preset frequencies or pulse widths by pressing the user button (PC13). All configuration is done via direct register access, without HAL or CMSIS drivers. This is ideal for learning low-level STM32 timer and GPIO control.

#### `TBA`
- **To Be Added**: Future examples will be added to demonstrate other STM32L4 features such as UART, I2C, DMA, FLASH, etc.

## License
This project is licensed under the **MIT License**. See the [LICENSE](./LICENSE) file for more details.
