#!/bin/bash

# Enhanced Docker image build and push script for STM32 development
# Supports both local development (ARM64) and CI/CD (x86_64)

set -e

# Configuration
DOCKER_USERNAME="dsavvari"
IMAGE_NAME="stm32-dev"
TAG="latest"
FULL_IMAGE_NAME="${DOCKER_USERNAME}/${IMAGE_NAME}:${TAG}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker and try again."
    exit 1
fi

# Check if logged in to Docker Hub
if ! docker info 2>/dev/null | grep -q "Username:"; then
    print_warning "Not logged in to Docker Hub. Attempting login..."
    echo "Please enter your Docker Hub credentials:"
    docker login
fi

echo "=================================="
echo "  STM32 Docker Image Builder"
echo "=================================="
echo "Target image: ${FULL_IMAGE_NAME}"
echo "Architecture: Multi-arch (linux/amd64, linux/arm64)"
echo ""

# Check if buildx is available
if ! docker buildx version >/dev/null 2>&1; then
    print_error "Docker buildx is not available. Please update Docker to a newer version."
    exit 1
fi

# Use the existing desktop-linux builder for multi-arch builds
print_status "Setting up Docker buildx for multi-architecture builds..."
docker buildx create --name stm32-builder --driver docker-container --use --bootstrap 2>/dev/null || {
    print_status "Builder already exists, using existing stm32-builder..."
    docker buildx use stm32-builder
}

# Build multi-architecture image
print_status "Building multi-architecture Docker image..."
print_status "This may take several minutes as it builds for both ARM64 and x86_64..."

docker buildx build \
    --platform linux/amd64,linux/arm64 \
    --tag ${FULL_IMAGE_NAME} \
    --push \
    . || {
    print_error "Failed to build and push Docker image"
    exit 1
}

print_success "Multi-architecture Docker image built and pushed successfully!"

# Verify the image
print_status "Verifying pushed image..."
docker buildx imagetools inspect ${FULL_IMAGE_NAME}

echo ""
print_success "Docker image ${FULL_IMAGE_NAME} is now available for:"
echo "  - Local development (ARM64 Mac)"
echo "  - GitHub Actions CI/CD (x86_64 Linux)"
echo ""
echo "GitHub Actions will automatically select the correct architecture."
echo ""
echo "Next steps:"
echo "1. Commit and push your changes to trigger the CI/CD pipeline"
echo "2. Monitor the GitHub Actions workflow in your repository"
