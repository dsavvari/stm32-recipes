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

#### `bareMetalSysTick`

- A bare-metal example demonstrating **SysTick timer** implementation on STM32L4S5VI (B-L4S5I-IOT01A). This example configures the SysTick timer for precise timing control and demonstrates synchronous LED blinking on three outputs: PA5 (LED1), PB14 (LED2), and PA0 (external LED/oscilloscope probe). The user can cycle through different blink frequencies (1000/500/300/250/200ms) by pressing the user button (PC13). All configuration is done via direct register access using CMSIS, making it ideal for learning low-level STM32 timing and interrupt handling.

#### `TBA`

- **To Be Added**: Future examples will be added to demonstrate other STM32L4 features such as UART, I2C, DMA, FLASH, etc.

## VS Code Integration

This project includes comprehensive VS Code tasks and launch configurations for seamless development in both environments:

### Dual Environment Support

The VS Code tasks are designed to work in two environments:

1. **Inside Docker Container**: When VS Code is running inside the Docker container (e.g., via Remote-Containers extension), use the standard tasks without "(Docker)" suffix.
2. **On Host System**: When VS Code is running on the host system, use the tasks with "(Docker)" suffix to execute commands inside the container.

### Build Tasks

**Standard Tasks (use inside Docker container or with local ARM toolchain):**

- **Build All Projects**: Build all STM32 projects (default build task)
- **Build Project**: Build a specific project (with dropdown selection)
- **Clean Build**: Clean all build artifacts
- **Configure CMake**: Configure CMake for Debug build
- **Initial Project Setup**: Complete setup including dependencies and CMake configuration
- **Setup Dependencies**: Setup third-party dependencies (STM32-CMake, STM32CubeL4)

**Docker Tasks (use from host system):**

- **Build All Projects (Docker)**: Build all STM32 projects inside Docker container
- **Build Project (Docker)**: Build a specific project inside Docker container
- **Clean Build (Docker)**: Clean build artifacts inside Docker container
- **Configure CMake (Docker)**: Configure CMake inside Docker container
- **Initial Project Setup (Docker)**: Complete setup inside Docker container
- **Setup Dependencies (Docker)**: Setup dependencies inside Docker container

### Hardware Tasks (Always run on Host)

- **Start OpenOCD Server**: Start OpenOCD server for STM32 debugging (requires OpenOCD installed on host)
- **Flash STM32**: Flash a binary to STM32 using OpenOCD (with project selection)

### Debug Configuration

- **GDB Debug STM32**: Generic debug configuration with project selection dropdown

**Note**: Hardware access tasks (OpenOCD, flashing, debugging) must always run on the host system where the ST-Link debugger is physically connected, regardless of whether the build environment is inside Docker or local.

## License

This project is licensed under the **MIT License**. See the [LICENSE](./LICENSE) file for more details.
