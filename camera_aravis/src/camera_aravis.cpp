

#include "camera_aravis/camera_aravis.hpp"

extern "C" {
#include "arv.h"
}

namespace camera_aravis
{
CameraAravis::CameraAravis(const rclcpp::NodeOptions& options) : Node("camera_aravis", options)
{
  // Print out some useful info.
  RCLCPP_INFO(get_logger(), "Attached cameras:");
  arv_update_device_list();
  uint n_interfaces = arv_get_n_interfaces();
  RCLCPP_INFO(get_logger(), "# Interfaces: %d", n_interfaces);

  uint n_devices = arv_get_n_devices();
  RCLCPP_INFO(get_logger(), "# Devices: %d", n_devices);
  for (uint i = 0; i < n_devices; i++)
    RCLCPP_INFO(get_logger(), "Device%d: %s", i, arv_get_device_id(i));

  if (n_devices == 0)
  {
    RCLCPP_INFO(get_logger(), "No cameras detected.");
    return;
  }
}

CameraAravis::~CameraAravis() = default;

}  // namespace camera_aravis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis::CameraAravis)
