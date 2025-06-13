# JSK Visualization ROS2 Port

This is a ROS2 port of the JSK Visualization package suite from ROS1. The original package provides visualization tools and plugins for RViz and rqt.

## Package Overview

### Ported Packages

1. **jsk_visualization** - Metapackage
2. **jsk_rqt_plugins** - rqt plugins for JSK tools
3. **jsk_rviz_plugins** - RViz plugins for JSK tools
4. **jsk_interactive_marker** - Interactive marker tools
5. **jsk_interactive** - Interactive functionality
6. **jsk_interactive_test** - Test package for interactive tools

## Major Changes for ROS2

### Build System
- **catkin** → **ament_cmake**
- Updated CMakeLists.txt files to use ament_cmake
- Updated package.xml files to format 3 with ROS2 dependencies

### Dependencies
- **rospy** → **rclpy**
- **roscpp** → **rclcpp**
- **tf** → **tf2**
- **actionlib** → **rclcpp_action**
- **dynamic_reconfigure** → (removed, needs alternative implementation)
- **rviz** → **rviz2**

### API Changes
- ROS1 node initialization → ROS2 node initialization
- ROS1 publishers/subscribers → ROS2 publishers/subscribers
- ROS1 services → ROS2 services
- ROS1 actions → ROS2 actions

### Message and Service Definitions
- Updated message generation to use `rosidl_default_generators`
- Updated runtime dependencies to use `rosidl_default_runtime`

## Current Status

### ✅ Completed
- Basic package structure conversion
- package.xml files updated to ROS2 format
- CMakeLists.txt files updated to ament_cmake
- Dependency mapping to ROS2 equivalents

### 🚧 In Progress / TODO
- **C++ Code Migration**: The C++ source files need significant updates:
  - Replace ROS1 headers with ROS2 equivalents
  - Update node initialization and lifecycle
  - Update publisher/subscriber API calls
  - Update service and action client/server implementations
  - Update tf usage to tf2

- **Python Code Migration**: Python scripts need updates:
  - Replace `rospy` with `rclpy`
  - Update node initialization
  - Update publisher/subscriber patterns
  - Update service and action patterns

- **Launch Files**: Convert XML launch files to Python launch files

- **Dynamic Reconfigure**: Find ROS2 alternative (parameters with callbacks)

- **Plugin System**: Update RViz and rqt plugin registration for ROS2

### ❌ Known Issues
- C++ compilation will fail until source code is updated
- Python scripts will fail until rospy→rclpy migration is complete
- Launch files need conversion from XML to Python format
- Some JSK-specific dependencies may not be available in ROS2

## Dependencies

The following external JSK packages need to be available in ROS2:
- jsk_footstep_msgs
- jsk_gui_msgs
- jsk_hark_msgs
- jsk_recognition_msgs
- jsk_recognition_utils
- jsk_topic_tools
- people_msgs
- view_controller_msgs

## Building

```bash
cd /path/to/ros2_workspace
colcon build --packages-select jsk_visualization
```

Note: Building will currently fail due to incomplete source code migration.

## Migration Strategy

1. **Phase 1**: Package structure (✅ Complete)
2. **Phase 2**: Message/Service definitions
3. **Phase 3**: Python code migration
4. **Phase 4**: C++ code migration
5. **Phase 5**: Plugin system updates
6. **Phase 6**: Launch file conversion
7. **Phase 7**: Testing and validation

## Contributing

To complete this migration:

1. Update C++ source files to use ROS2 APIs
2. Update Python scripts to use rclpy
3. Convert launch files to Python format
4. Update plugin configurations for ROS2
5. Test functionality with ROS2 Jazzy

## Original Repository

Original ROS1 package: https://github.com/jsk-ros-pkg/jsk_visualization

## License

BSD License (same as original package)