#!/bin/bash

# Script to push STM32 development Docker image to Docker Hub
# Usage: ./push-docker-image.sh [your-dockerhub-username]

set -e

DOCKERHUB_USERNAME=${1:-"YOUR_DOCKERHUB_USERNAME"}
LOCAL_IMAGE="dev_stm32:latest"
REMOTE_IMAGE="${DOCKERHUB_USERNAME}/stm32-dev:latest"

if [ "$DOCKERHUB_USERNAME" = "YOUR_DOCKERHUB_USERNAME" ]; then
    echo "❌ Error: Please provide your Docker Hub username"
    echo "Usage: $0 <your-dockerhub-username>"
    echo "Example: $0 john_doe"
    exit 1
fi

echo "🚀 Pushing STM32 Docker image to Docker Hub..."
echo "Local image: $LOCAL_IMAGE"
echo "Remote image: $REMOTE_IMAGE"
echo ""

# Check if local image exists
if ! docker image inspect "$LOCAL_IMAGE" > /dev/null 2>&1; then
    echo "❌ Error: Local image '$LOCAL_IMAGE' not found"
    echo "Please make sure your dev_stm32 image exists locally"
    echo "You can check with: docker images | grep dev_stm32"
    exit 1
fi

# Tag the image for Docker Hub
echo "🏷️  Tagging image..."
docker tag "$LOCAL_IMAGE" "$REMOTE_IMAGE"

# Login to Docker Hub (user will be prompted for credentials)
echo "🔐 Logging in to Docker Hub..."
echo "You will be prompted for your Docker Hub credentials:"
docker login

# Push the image
echo "📤 Pushing image to Docker Hub..."
docker push "$REMOTE_IMAGE"

echo ""
echo "✅ Success! Your image is now available at:"
echo "   $REMOTE_IMAGE"
echo ""
echo "🔧 Next steps:"
echo "1. Update .github/workflows/build-advanced.yml"
echo "2. Replace 'YOUR_DOCKERHUB_USERNAME' with '$DOCKERHUB_USERNAME'"
echo "3. Commit and push your changes"
echo ""
echo "The workflow will use: $REMOTE_IMAGE"
