#pragma once

#include "rclcpp/rclcpp.hpp"

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
public:
  explicit CameraAravis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraAravis();
};

}  // end namespace camera_aravis
