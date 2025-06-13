#include <rclcpp/rclcpp.hpp>
#include <jsk_interactive_marker/transformable_interactive_server.h>

using namespace jsk_interactive_marker;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  TransformableInteractiveServer* ti_server = new TransformableInteractiveServer();

  ti_server->run();
  
  rclcpp::shutdown();
  return 0;
}
