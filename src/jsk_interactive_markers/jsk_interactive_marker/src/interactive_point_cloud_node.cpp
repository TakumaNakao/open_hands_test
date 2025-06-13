#include <rclcpp/rclcpp.hpp>
#include <jsk_interactive_marker/interactive_point_cloud.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  InteractivePointCloud interactive_point_cloud("interactive_manipulation_snapshot",
						"interactive_point_cloud", "interactive_manipulation_snapshot_server");
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin();
  rclcpp::shutdown();
  return 0;
}

