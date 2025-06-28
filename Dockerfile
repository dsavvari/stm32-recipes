# STM32 Development Environment
# Multi-architecture Docker image for local development (ARM64) and CI/CD (AMD64)
FROM ubuntu:22.04

# Build arguments for architecture detection
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH

# Set a working directory
WORKDIR /repo

ENV DEBIAN_FRONTEND=noninteractive

# Install common system dependencies
RUN apt-get update && apt-get install -y \
  build-essential \
  tar \
  cmake \
  ninja-build \
  git \
  wget \
  curl \
  unzip \
  python3 \
  python3-pip \
  gdb \
  gdb-multiarch \
  libusb-1.0-0-dev \
  pkg-config \
  stlink-tools \
  gcc-aarch64-linux-gnu \
  gcc-arm-linux-gnueabihf \
  vim \
  clang \
  clangd \
  clang-format \
  clang-tidy \
  openocd \
  stlink-tools \
&& apt-get clean

# Install xPack GCC ARM toolchain with architecture detection
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        TOOLCHAIN_ARCH="linux-x64"; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        TOOLCHAIN_ARCH="linux-arm64"; \
    else \
        echo "Unsupported architecture: $TARGETARCH" && exit 1; \
    fi && \
    wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1/xpack-arm-none-eabi-gcc-14.2.1-1.1-${TOOLCHAIN_ARCH}.tar.gz && \
    tar -xzf xpack-arm-none-eabi-gcc-14.2.1-1.1-${TOOLCHAIN_ARCH}.tar.gz -C /opt && \
    ln -s /opt/xpack-arm-none-eabi-gcc-*/bin/* /usr/local/bin/ && \
    rm xpack-arm-none-eabi-gcc-14.2.1-1.1-${TOOLCHAIN_ARCH}.tar.gz

# Verify toolchain installation
RUN arm-none-eabi-gcc --version && \
    arm-none-eabi-size --version && \
    cmake --version && \
    echo "Architecture: $TARGETARCH" && \
    echo "Platform: $TARGETPLATFORM"

# Default command
CMD ["/bin/bash"]
