
#include "camera_aravis/camera_aravis.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_aravis::CameraAravis>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
