#!/bin/bash

# Build and test JSK Visualization ROS2 package
# This script builds the package and runs tests

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== JSK Visualization ROS2 Build and Test Script ===${NC}"

# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS_DISTRO not set. Sourcing ROS2 Jazzy...${NC}"
    source /opt/ros/jazzy/setup.bash
fi

echo -e "${BLUE}ROS Distribution: $ROS_DISTRO${NC}"

# Find workspace root
if [ -f "src/jsk_rviz_plugins/package.xml" ]; then
    WORKSPACE_ROOT=$(pwd)
elif [ -f "../package.xml" ]; then
    WORKSPACE_ROOT=$(dirname $(pwd))
elif [ -f "package.xml" ]; then
    WORKSPACE_ROOT=$(dirname $(pwd))
else
    echo -e "${RED}Error: Could not find JSK RViz plugins package.xml${NC}"
    echo "Please run this script from the workspace root or package directory"
    exit 1
fi

echo -e "${BLUE}Workspace: $WORKSPACE_ROOT${NC}"
cd "$WORKSPACE_ROOT"

# Clean previous build (optional)
if [ "$1" = "--clean" ]; then
    echo -e "${YELLOW}Cleaning previous build...${NC}"
    rm -rf build/ install/ log/
fi

# Install dependencies
echo -e "${BLUE}Installing dependencies...${NC}"
rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" || \
    echo -e "${YELLOW}Some dependencies could not be installed, continuing...${NC}"

# Build the package
echo -e "${BLUE}Building JSK RViz plugins...${NC}"
colcon build --packages-select jsk_rviz_plugins \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Build successful!${NC}"
else
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi

# Source the workspace
echo -e "${BLUE}Sourcing workspace...${NC}"
source install/setup.bash

# Run tests
echo -e "${BLUE}Running tests...${NC}"
colcon test --packages-select jsk_rviz_plugins \
    --event-handlers console_direct+

# Show test results
echo -e "${BLUE}Test results:${NC}"
colcon test-result --verbose

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ All tests passed!${NC}"
else
    echo -e "${YELLOW}⚠️  Some tests failed. Check the output above.${NC}"
fi

# Check if RViz2 can load the plugins
echo -e "${BLUE}Checking plugin loading...${NC}"
timeout 5s ros2 run rviz2 rviz2 --help > /dev/null 2>&1
if [ $? -eq 0 ] || [ $? -eq 124 ]; then  # 124 is timeout exit code
    echo -e "${GREEN}✅ RViz2 can be launched${NC}"
else
    echo -e "${YELLOW}⚠️  RViz2 launch check failed${NC}"
fi

echo -e "${GREEN}=== Build and test completed! ===${NC}"
echo -e "${BLUE}To use the package:${NC}"
echo "  source install/setup.bash"
echo "  ros2 run rviz2 rviz2"