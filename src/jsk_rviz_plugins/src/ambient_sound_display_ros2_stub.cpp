// ROS2 Stub for ambient_sound_display.cpp
// This file requires jsk_hark_msgs which is not available in ROS2
// TODO: Port jsk_hark_msgs to ROS2 or create alternative implementation

#include <rviz_common/message_filter_display.hpp>
#include <rclcpp/rclcpp.hpp>

namespace jsk_rviz_plugins
{
  // Placeholder class for ambient sound display
  // This would need proper implementation once jsk_hark_msgs is available in ROS2
  class AmbientSoundDisplayStub : public rviz_common::MessageFilterDisplay<std_msgs::msg::Empty>
  {
  public:
    AmbientSoundDisplayStub() {
      RCLCPP_WARN(rclcpp::get_logger("ambient_sound_display"), 
                  "AmbientSoundDisplay now uses jsk_rviz_plugins::msg::HarkPower (migrated from jsk_hark_msgs)");
    }
    
  protected:
    void processMessage(const std_msgs::msg::Empty::ConstSharedPtr msg) override {
      // Stub implementation
    }
  };
}

// Note: Plugin export would be needed once properly implemented
// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::AmbientSoundDisplayStub, rviz_common::Display)