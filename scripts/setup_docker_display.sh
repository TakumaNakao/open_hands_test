#!/bin/bash

# Setup X11 forwarding for Docker containers
# This script prepares the display for GUI applications in Docker

set -e

echo "Setting up X11 forwarding for Docker..."

# Create X11 authentication file
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "X11 authentication file created: $XAUTH"

# Allow X11 connections from Docker
xhost +local:docker

echo "X11 forwarding setup complete!"
echo ""
echo "You can now run GUI applications in Docker containers."
echo "Example usage:"
echo "  docker-compose up jsk-visualization-rviz"
echo "  docker run -it --rm -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw jsk-visualization-ros2:jazzy"