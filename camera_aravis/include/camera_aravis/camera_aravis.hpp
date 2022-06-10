#pragma once

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <vector>
#include <unordered_map>

extern "C" {
#include "arv.h"
}

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
public:
  explicit CameraAravis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraAravis();

private:
  bool verbose_;

  ArvCamera * p_camera_;
  ArvDevice * p_device_;
  gint num_streams_;
  std::vector<ArvStream *> p_streams_;
  std::vector<std::string> stream_names_;

  std::unordered_map<std::string, const bool> implemented_features_;

  void setup_parameters();
  void discover_features();
  void get_num_streams();
};

}  // end namespace camera_aravis
