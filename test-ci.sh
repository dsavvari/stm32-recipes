#!/bin/bash

# Local CI test script - simulates the GitHub Actions workflow
# This allows you to test the build process locally before pushing

set -e  # Exit on any error

echo "🚀 Starting local STM32 CI test..."
echo "=================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -f "thirdparty.sh" ]; then
    echo -e "${RED}❌ Error: Please run this script from the project root directory${NC}"
    exit 1
fi

# Check for ARM toolchain
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo -e "${RED}❌ Error: ARM GCC toolchain not found${NC}"
    echo "Please install arm-none-eabi-gcc or run this inside the Docker container"
    exit 1
fi

echo -e "${YELLOW}📋 Checking toolchain versions...${NC}"
arm-none-eabi-gcc --version | head -1
cmake --version | head -1

echo -e "${YELLOW}📦 Setting up dependencies...${NC}"
chmod +x ./thirdparty.sh
./thirdparty.sh

echo -e "${YELLOW}⚙️  Configuring CMake (Release build)...${NC}"
mkdir -p build-test
cd build-test
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../repos/stm32-cmake/cmake/stm32_gcc.cmake

echo -e "${YELLOW}🔨 Building all projects...${NC}"
cmake --build . --parallel $(nproc 2>/dev/null || echo 4)

echo -e "${YELLOW}📊 Displaying binary sizes...${NC}"
echo "| Project | Text | Data | BSS | Total | Hex |"
echo "|---------|------|------|-----|-------|-----|"

BUILD_SUCCESS=true
for project in bareMetalBlink bareMetalPWM bareMetalSysTick; do
    if [ -f "${project}/stm32l4-${project}.elf" ]; then
        size_output=$(arm-none-eabi-size "${project}/stm32l4-${project}.elf" | tail -n 1)
        text=$(echo $size_output | awk '{print $1}')
        data=$(echo $size_output | awk '{print $2}')
        bss=$(echo $size_output | awk '{print $3}')
        dec=$(echo $size_output | awk '{print $4}')
        hex=$(echo $size_output | awk '{print $5}')
        echo "| $project | $text | $data | $bss | $dec | $hex |"
    else
        echo -e "${RED}❌ Missing binary: ${project}/stm32l4-${project}.elf${NC}"
        BUILD_SUCCESS=false
    fi
done

cd ..

if [ "$BUILD_SUCCESS" = true ]; then
    echo -e "${GREEN}✅ All projects built successfully!${NC}"
    echo -e "${GREEN}📁 Binaries available in: build-test/*/stm32l4-*.elf${NC}"
    echo ""
    echo -e "${YELLOW}🗂️  Generated files:${NC}"
    find build-test -name "stm32l4-*.elf" -o -name "stm32l4-*.bin" -o -name "stm32l4-*.hex" | sort
else
    echo -e "${RED}❌ Build failed! Check the output above for details.${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}🎉 Local CI test completed successfully!${NC}"
echo -e "${YELLOW}💡 You can now safely push to trigger the GitHub Actions workflow.${NC}"
