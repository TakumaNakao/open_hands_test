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

## Migration Progress

### ✅ Completed
- [x] Package structure analysis
- [x] Package.xml files converted to format 3
- [x] CMakeLists.txt files updated to ament_cmake
- [x] Basic workspace setup
- [x] Python script migration (rospy → rclpy) - 7 files migrated
- [x] C++ header migration (basic ROS2 API updates) - 2 files
- [x] Setup.py configuration for Python entry points
- [x] Message definitions (already ROS2 compatible)

### 🔄 In Progress
- [x] C++ source code migration (6 files completed, more pending)
- [x] Plugin configurations for RViz2 (base classes updated)
- [x] Launch files (1 sample converted, more pending)
- [ ] Complex C++ files with external dependencies

### ⏳ Pending
- [ ] Build testing (requires proper ROS2 environment)
- [ ] Runtime testing
- [ ] Documentation updates
- [ ] Missing dependency resolution (jsk_hark_msgs, jsk_gui_msgs, etc.)
- [ ] Complete remaining C++ implementation files

### Python Scripts Migrated
1. `overlay_sample.py` - Basic overlay text publishing
2. `piechart_sample.py` - Pie chart visualization
3. `static_overlay_text.py` - Static text overlay with parameters
4. `overlay_text_interface.py` - Text interface with parameter system
5. `rosconsole_overlay_text.py` - Console log overlay
6. `float32_to_overlay_text.py` - Float32 to text conversion
7. `pictogram.py` - Pictogram cycling sample

### C++ Headers Migrated
1. `overlay_utils.h` - Basic ROS2 includes and smart pointers
2. `overlay_text_display.h` - RViz2 display plugin headers

### C++ Implementation Files Migrated
1. `overlay_text_display.cpp` - Complete ROS2 migration with subscription patterns
2. `pie_chart_display.cpp` - Property types and subscription methods updated
3. `close_all_tool.cpp` - Complete migration to rviz_common::Tool
4. `open_all_tool.cpp` - Complete migration to rviz_common::Tool
5. `image_transport_hints_property.cpp` - Property types and transport hints API
6. `object_fit_operator.cpp` - Partial migration (includes, constructor, package path)

### Plugin System Updates
- Plugin description XML files updated for RViz2 base classes
- Message definitions updated (Header references)
- Sample launch file converted to Python format

### Migration Patterns Applied

#### Python (rospy → rclpy)
```python
# ROS1 Pattern
import rospy
rospy.init_node("node_name")
pub = rospy.Publisher("topic", MsgType, queue_size=10)
sub = rospy.Subscriber("topic", MsgType, callback)
timer = rospy.Timer(rospy.Duration(0.1), timer_callback)
param = rospy.get_param("~param", default_value)

# ROS2 Pattern
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        self.pub = self.create_publisher(MsgType, 'topic', 10)
        self.sub = self.create_subscription(MsgType, 'topic', self.callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('param', default_value)
        param = self.get_parameter('param').value
```

#### C++ Headers (ROS1 → ROS2)
```cpp
// ROS1
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
```

### ❌ Known Issues
- C++ compilation will fail until source code is updated
- Some JSK-specific dependencies may not be available in ROS2
- Build testing requires proper ROS2 environment setup

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