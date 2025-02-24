#!/bin/bash

# Get the script's directory (relative path handling)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define environment variables
export PROJECT_ROOT="$SCRIPT_DIR"
export REPO_DIR="$PROJECT_ROOT/repos"

#Define ARM toolchain location
export ARM_TOOLCHAIN_PATH="$PROJECT_ROOT/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin"
export ARM_CROSSTOOL_PREFIX=arm-none-eabi
export ARM_STM32_FAMILY="L4"

#Toolchain required - stop here if not found
if [ ! -d "$ARM_TOOLCHAIN_PATH" ]; then
    echo "❌ ERROR: can't find toolchain under location $ARM_TOOLCHAIN_PATH"
    exit 1  # Exit on failure
else
    echo "✅ found toolchain under location $ARM_TOOLCHAIN_PATH"
fi

#PATH
export PATH="$PROJECT_ROOT/bin:$ARM_TOOLCHAIN_PATH:$PATH"

# Create repos directory if it doesn't exist
mkdir -p "$REPO_DIR"

# Define repositories (name -> Git URL)
REPOS=(
    "stm32-cmake|https://github.com/ObKo/stm32-cmake.git"
    "STM32Cube$ARM_STM32_FAMILY|https://github.com/STMicroelectronics/STM32Cube$ARM_STM32_FAMILY.git"
)
# Clone repositories if they don’t exist
for repo_entry in "${REPOS[@]}"; do
    # Split name and URL
    repo_name="${repo_entry%%|*}"
    repo_url="${repo_entry##*|}"

    REPO_PATH="$REPO_DIR/$repo_name"
    REPO_EXPORT="$repo_name"
    if [ ! -d "$REPO_PATH/.git" ]; then
        echo "Cloning $repo_url into $REPO_PATH..."
        git clone --recurse-submodules  "$repo_url" "$REPO_PATH"

        # Check the return code of git clone
        if [ $? -ne 0 ]; then
            echo "❌ ERROR: Failed to clone $repo_url. Removing incomplete directory."
            rm -rf "$REPO_PATH"
            exit 1  # Exit on failure
        else
            echo "✅ Repository $repo_name cloned."
        fi
    else
        echo "✅ Repository $repo_name already exists. Skipping clone."
    fi
done

export STM32_CMAKE="$REPO_DIR/stm32-cmake"
export STM32_TOOLCHAIN_PATH="$ARM_TOOLCHAIN_PATH"
export STM32_TARGET_TRIPLET="$ARM_CROSSTOOL_PREFIX"
export STM32_CUBE_L4_PATH="$REPO_DIR/STM32Cube$ARM_STM32_FAMILY"

echo "Environment setup complete."

# Optional: Print the variables to confirm
echo "STM32_TOOLCHAIN_PATH=$STM32_TOOLCHAIN_PATH"
echo "STM32_TARGET_TRIPLET=$STM32_TARGET_TRIPLET"
echo "STM32_CUBE_L4_PATH=$STM32_CUBE_L4_PATH"
