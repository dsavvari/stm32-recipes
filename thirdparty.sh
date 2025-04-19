#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_ROOT="$SCRIPT_DIR"
export REPO_DIR="$PROJECT_ROOT/repos"
export ARM_STM32_FAMILY="L4"

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
