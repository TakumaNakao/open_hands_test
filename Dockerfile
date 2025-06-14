FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libboost-all-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    qtbase5-dev \
    qttools5-dev \
    libqt5opengl5-dev \
    libyaml-cpp-dev \
    libconsole-bridge-dev \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (conditionally)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Create workspace
WORKDIR /ros2_ws

# Copy only the jsk_rviz_plugins package
COPY src/jsk_rviz_plugins/ src/jsk_rviz_plugins/

# Install ROS2 packages manually (since rosdep may not find jazzy packages)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz-rendering \
    ros-${ROS_DISTRO}-rviz-ogre-vendor \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-resource-retriever \
    python3-scipy \
    python3-sklearn \
    && rm -rf /var/lib/apt/lists/* || \
    echo "Some ROS packages could not be installed, continuing..."

# Install dependencies for jsk_rviz_plugins only
RUN rosdep install --from-paths src/jsk_rviz_plugins --ignore-src -r -y \
    --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers jsk_gui_msgs jsk_recognition_utils jsk_tools view_controller_msgs people_msgs jsk_footstep_msgs jsk_hark_msgs jsk_recognition_msgs jsk_topic_tools posedetection_msgs jsk_data pr2_description urdfdom_py image_publisher joint_state_publisher" || \
    echo "Some dependencies could not be installed, continuing..."

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select jsk_rviz_plugins \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

# Source workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Add labels for metadata
LABEL maintainer="JSK Visualization ROS2 Team"
LABEL description="JSK Visualization package for ROS2 Jazzy"
LABEL version="1.0.0"
LABEL ros.distro="jazzy"