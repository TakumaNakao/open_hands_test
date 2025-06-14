cmake_minimum_required(VERSION 3.8)
project(jsk_rviz_plugins)

# Set CMake policies to suppress warnings
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
# Note: view_controller_msgs not available in ROS2 Jazzy
# Note: jsk_* packages replaced with local message definitions
# Note: people_msgs not available in ROS2 Jazzy
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)

# Qt5 for GUI components
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OverlayText.msg"
  "msg/OverlayMenu.msg"
  "msg/TransformableMarkerOperate.msg"
  "msg/ObjectFitCommand.msg"
  "msg/Pictogram.msg"
  "msg/PictogramArray.msg"
  "msg/RecordCommand.msg"
  "msg/StringStamped.msg"
  "msg/HarkPower.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxArray.msg"
  "msg/SegmentArray.msg"
  "msg/SimpleOccupancyGrid.msg"
  "msg/SimpleOccupancyGridArray.msg"
  "msg/PolygonArray.msg"
  "msg/TorusArray.msg"
  "msg/HumanSkeletonArray.msg"
  "srv/RequestMarkerOperate.srv"
  "srv/EusCommand.srv"
  "srv/Screenshot.srv"
  "srv/YesNo.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Python package installation is handled by setup.py
# Note: ament_python_install_package conflicts with setup.py approach

# Python scripts installation is handled by setup.py

# Install plugin configuration
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch samples config icons resources
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Note: C++ plugin compilation would require significant porting effort
# For now, we focus on the Python components and basic structure

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()