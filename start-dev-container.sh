#!/bin/bash

# STM32 Development Container Manager
# Uses the multi-architecture Docker image from Docker Hub

CONTAINER_NAME="dev_stm32"
IMAGE_NAME="dsavvari/stm32-dev:latest"
PROJECT_DIR=$(pwd)

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    print_info "Container '${CONTAINER_NAME}' already exists"

    # Check if it's running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_info "Container is already running. Attaching..."
        docker exec -it ${CONTAINER_NAME} /bin/bash
    else
        print_info "Starting existing container..."
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} /bin/bash
    fi
else
    print_info "Creating and starting new container '${CONTAINER_NAME}'..."
    docker run -it \
        --name ${CONTAINER_NAME} \
        -v "${PROJECT_DIR}:/repo" \
        -w /repo \
        ${IMAGE_NAME} \
        /bin/bash
fi
