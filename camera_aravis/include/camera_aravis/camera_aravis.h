#pragma once

// Std
#include <memory>
#include <vector>
#include <unordered_map>

// Aravis
extern "C" {
#include "arv.h"
}

// ROS
#include "rclcpp/rclcpp.hpp"

// camera_aravis
#include "error.hpp"

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
  //--- METHOD DECLARATION ---//

public:
  explicit CameraAravis(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraAravis() override;

private:
  void setup_parameters();
  void discover_features();
  void get_num_streams();

  //--- MEMBER DECLARATION ---//
private:
  rclcpp::Logger logger_;

  bool verbose_;

  GuardedGError err_;

  ArvCamera * p_camera_;
  ArvDevice * p_device_;
  gint num_streams_;
  std::vector<ArvStream *> p_streams_;
  std::vector<std::string> stream_names_;

  std::unordered_map<std::string, const bool> implemented_features_;
};

}  // namespace camera_aravis
