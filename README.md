# STM32 Component Setup Recipes

## About
This repository provides a collection of examples for setting up various **STM32** components using **bare-metal**, **CMSIS**, and **HAL** frameworks. The examples are tailored for the **STM32L4** series, specifically using the **B-L4S5I-IOT01A Discovery Board**.

**🚀 Features:**

- ✅ **Multi-architecture Docker support** (ARM64 Mac + x86_64 CI/CD)
- ✅ **Complete CI/CD pipeline** with GitHub Actions
- ✅ **VS Code integration** with comprehensive tasks and debugging
- ✅ **Automated notifications** (GitHub Issues, PR comments, build artifacts)
- ✅ **Consistent development environment** across local and cloud builds

## Quick Start

### Option 1: Docker Development (Recommended)

The fastest way to get started with a consistent, pre-configured environment:

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/stm32-recipes.git
cd stm32-recipes

# Start development container (downloads multi-arch image automatically)
./start-dev-container.sh

# Inside container: Setup dependencies and build
./thirdparty.sh
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build .
```

### Option 2: Local Development

For local development with your own ARM toolchain:

```bash
# Install dependencies
# - ARM GNU Toolchain: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# - CMake 3.15+

# Setup and build
./thirdparty.sh
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build .
```

## Development Environment

### Docker-Based Development

This repository includes a **multi-architecture Docker image** (`dsavvari/stm32-dev:latest`) that works on:

- **ARM64 Macs** (Apple Silicon)
- **x86_64 Linux** (GitHub Actions, Intel/AMD)

#### Container Management

```bash
# Start/attach to development container
./start-dev-container.sh

# The script automatically:
# - Creates container if it doesn't exist
# - Starts container if stopped
# - Attaches to running container
```

#### Docker Image Management

```bash
# Build and push multi-architecture image (maintainers only)
./build-and-push-docker.sh

# Or push existing local image
./push-docker-image.sh
```

### Local Development Requirements

If not using Docker, ensure you have:

- **ARM Toolchain**: [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
- **CMake 3.15+**: For build system
- **Git**: For dependency management

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

## GitHub Actions CI/CD

![Build Status](https://github.com/YOUR_USERNAME/stm32-recipes/workflows/STM32%20Build%20and%20Test/badge.svg)

This repository includes a **production-ready CI/CD pipeline** with GitHub Actions that provides:

### 🚀 **Advanced CI/CD Features**

- ✅ **Multi-architecture Docker support**: Uses same image locally (ARM64) and in CI (x86_64)
- ✅ **Fast builds**: Pre-built Docker image eliminates toolchain installation time
- ✅ **Build caching**: Dependencies cached between runs for speed
- ✅ **Comprehensive notifications**: GitHub Issues, PR comments, build summaries
- ✅ **Artifact management**: ELF files uploaded for download
- ✅ **Build status reporting**: Real-time status updates and detailed summaries
- ✅ **Scheduled builds**: Weekly health checks to catch environment drift

### 📋 **Workflow Details**

**Main Workflow**: `STM32 Build and Test`

- **Triggers**: Push to master/main, Pull Requests, Weekly schedule (Sundays 6 AM UTC)
- **Environment**: Custom Docker image (`dsavvari/stm32-dev:latest`)
- **Projects Built**: bareMetalBlink, bareMetalPWM, bareMetalSysTick
- **Outputs**: Build status, binary sizes, downloadable artifacts

**Build Process:**
1. Checkout repository with submodules
2. Load Docker image (pre-built with ARM toolchain)
3. Cache third-party dependencies
4. Setup STM32-CMake and STM32CubeL4
5. Configure CMake build system
6. Build all projects in parallel
7. Collect binary size information
8. Generate build summary
9. Upload artifacts on success
10. Create GitHub Issue on failure (master branch only)

### 🔔 **Notification System**

The CI system provides multiple notification channels:

#### **Automatic Notifications**
- ✅ **GitHub Issues**: Created for failed builds on master branch
- ✅ **PR Comments**: Added to pull requests for build failures
- ✅ **Build Summaries**: Detailed reports in GitHub Actions UI
- ✅ **Build Artifacts**: ELF files available for download after successful builds

#### **Optional Notifications**
- � **Email**: GitHub's built-in email notifications
- 💬 **Slack/Discord/Teams**: Custom webhook integration
- 📱 **Mobile**: Via GitHub mobile app

**Quick Setup for Basic Notifications:**
1. Repository → Click "Watch" → "All Activity"
2. GitHub Settings → Notifications → Enable "Actions"

**Advanced Notifications Setup:**
See [NOTIFICATIONS.md](NOTIFICATIONS.md) for detailed setup instructions for Slack, Discord, Teams, and custom webhooks.

### 📊 **Build Metrics**

Each successful build provides:

- **Binary Size Analysis**: Text, data, BSS, and total memory usage
- **Build Performance**: Timing and caching efficiency
- **Project Status**: Individual project build results
- **Environment Info**: Toolchain versions and build configuration

Example build summary:

```text
STM32 Build Summary
==================
Commit: abc123def
Branch: master
Build Type: Release

✅ Build Status: SUCCESS

Binary Sizes:
| Project          | Text   | Data | BSS  | Total | Hex  |
|------------------|--------|------|------|-------|------|
| bareMetalBlink   | 1060   | 16   | 1972 | 3048  | be8  |
| bareMetalPWM     | 1248   | 20   | 1972 | 3240  | ca8  |
| bareMetalSysTick | 1156   | 16   | 1972 | 3144  | c48  |
```

### 🛠 **Docker Management Scripts**

This repository includes several helper scripts for managing the Docker development environment:

#### `start-dev-container.sh`

**Purpose**: One-command solution for Docker-based development
**Usage**: `./start-dev-container.sh`

Features:

- Automatically pulls the latest multi-arch image if not present
- Creates container if it doesn't exist
- Starts container if stopped
- Attaches to running container
- Mounts project directory at `/repo`
- Uses consistent container name (`dev_stm32`) for VS Code task compatibility

#### `build-and-push-docker.sh`

**Purpose**: Build and push multi-architecture Docker images (maintainers only)
**Usage**: `./build-and-push-docker.sh`

Features:

- Creates buildx builder for multi-arch support
- Builds for both ARM64 and x86_64 architectures
- Pushes to Docker Hub with proper manifest
- Verifies image availability
- Cleans up temporary builders

#### `push-docker-image.sh`

**Purpose**: Push existing local Docker image to Docker Hub
**Usage**: `./push-docker-image.sh`

Features:

- Tags local image for Docker Hub
- Handles authentication
- Pushes single-architecture image

### 💻 **Development Workflow**

#### **For Regular Development:**

```bash
# Start development environment
./start-dev-container.sh

# Inside container: Build projects
cd build
cmake --build . --target stm32l4-bareMetalBlink

# Or build all projects
cmake --build .
```

#### **For Contributors/Maintainers:**

```bash
# Update Docker image (after Dockerfile changes)
./build-and-push-docker.sh

# Commit and push changes
git add .
git commit -m "feat: Add new feature"
git push  # Triggers CI/CD automatically
```

#### **VS Code Integration:**

- Open repository in VS Code
- Use Command Palette: "Tasks: Run Task"
- Available tasks:
  - **Docker tasks**: For host-based development
  - **Standard tasks**: For container-based development
  - **Hardware tasks**: For debugging and flashing (host only)

### 🛠 **Local CI Testing**

Test CI behavior locally using the same Docker environment:

```bash
# Use the same image as CI/CD
./start-dev-container.sh

# Run the same build steps as CI
./thirdparty.sh
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_TOOLCHAIN_FILE=../repos/stm32-cmake/cmake/stm32_gcc.cmake \
         -DSTM32_CHIP=STM32L4S5VI
cmake --build . --parallel $(nproc)

# Check binary sizes like CI does
for project in bareMetalBlink bareMetalPWM bareMetalSysTick; do
  if [ -f "${project}/stm32l4-${project}.elf" ]; then
    arm-none-eabi-size "${project}/stm32l4-${project}.elf"
  fi
done
```

## Troubleshooting

### Docker Issues

**Problem**: `Error response from daemon: pull access denied`
**Solution**: Make sure you're logged into Docker Hub: `docker login`

**Problem**: `Multi-platform build is not supported for the docker driver`
**Solution**: The build script automatically creates a proper buildx builder. If issues persist, run:
```bash
docker buildx create --name stm32-builder --use --bootstrap
```

**Problem**: Container startup issues

**Solution**:

```bash
# Clean up and restart
docker stop dev_stm32
docker rm dev_stm32
./start-dev-container.sh
```

### Build Issues

**Problem**: `arm-none-eabi-gcc: command not found`

**Solution**:

- If using Docker: Ensure you're in the container (`./start-dev-container.sh`)
- If local: Install ARM GNU Toolchain and ensure it's in PATH

**Problem**: CMake configuration fails

**Solution**:

```bash
# Clean and reconfigure
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

**Problem**: Third-party dependencies missing

**Solution**:

```bash
# Re-run dependency setup
./thirdparty.sh
```

### CI/CD Issues

**Problem**: GitHub Actions failing with shell errors
**Solution**: The workflow uses POSIX-compliant shell syntax. Avoid bash-specific operators like `==` in shell scripts.

**Problem**: Docker image pull failures in CI
**Solution**: Ensure the Docker image `dsavvari/stm32-dev:latest` exists and is public on Docker Hub.

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes and test locally using the Docker environment
4. Commit with clear messages: `git commit -m "feat: Add amazing feature"`
5. Push to your fork: `git push origin feature/amazing-feature`
6. Create a Pull Request

**Development Guidelines:**

- Use the provided Docker environment for consistency
- Test builds locally before pushing
- Follow conventional commit messages
- Update documentation for new features
- Ensure CI/CD passes before merging

## License

This project is licensed under the **MIT License**. See the [LICENSE](./LICENSE) file for more details.

---

**🚀 Happy STM32 Development!**

For questions, issues, or contributions, please use the GitHub issue tracker or discussions.
