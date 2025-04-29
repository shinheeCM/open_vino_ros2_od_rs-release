#!/bin/bash

# Stop on error
set -e

# https://github.com/microsoft/onnxruntime/releases/download/v1.15.1/onnxruntime-linux-x64-gpu-1.15.1.tgz
# Variables
BUILD_DIR="build"

# Colors
GREEN='\033[1;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}📁 Creating build directory...${NC}"
mkdir -p $BUILD_DIR
cd $BUILD_DIR

echo -e "${GREEN}⚙️  Running CMake...${NC}"
cmake ..

echo -e "${GREEN}🔨 Building project...${NC}"
make -j$(nproc)

echo -e "${GREEN}✅ Build complete! Run your app using: ./$BUILD_DIR/<your_executable_name>${NC}"
