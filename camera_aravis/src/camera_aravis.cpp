

#include "camera_aravis/camera_aravis.hpp"

#include <chrono>

namespace camera_aravis
{
CameraAravis::CameraAravis(const rclcpp::NodeOptions& options) : Node("camera_aravis", options), verbose_(declare_parameter<bool>("verbose", false))
{
  setup_parameters();

  // Print out some useful info.
  RCLCPP_INFO(get_logger(), "Attached cameras:");
  arv_update_device_list();
  auto n_interfaces = arv_get_n_interfaces();
  RCLCPP_INFO(get_logger(), "# Interfaces: %d", n_interfaces);

  auto n_devices = arv_get_n_devices();
  RCLCPP_INFO(this->get_logger(), "# Devices: %d", n_devices);
  for (uint i = 0; i < n_devices; i++)
    RCLCPP_INFO(this->get_logger(), "Device %d: %s", i, arv_get_device_id(i));

  if (n_devices == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No cameras detected. Shutting down...");
    // TODO: Shutdown?
    rclcpp::shutdown();
    return;
  }

  // Get the camera guid as a parameter or use the first device.
  std::string guid = get_parameter("guid").as_string();

  // Open the camera, and set it up.
  do
  {
    if (guid == "")
    {
      RCLCPP_INFO(this->get_logger(), "Opening: (any)");
      p_camera_ = arv_camera_new(nullptr);
    }
    else
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Opening: " << guid);
      p_camera_ = arv_camera_new(guid.c_str());
    }

    if (!p_camera_)
    {
      RCLCPP_WARN(this->get_logger(), "Unable to open camera. Retrying...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  } while (!p_camera_);

  p_device_ =arv_camera_get_device(p_camera_);
  RCLCPP_INFO(this->get_logger(), "Successfully opened: %s-%s", arv_camera_get_vendor_name(p_camera_),
              arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber"));

  // See which features exist in this camera device
  discover_features();

   // Check the number of streams for this camera
  get_num_streams();
  RCLCPP_INFO(get_logger(), "Supported stream channels: %i", static_cast<int>(num_streams_));

  auto const stream_names_ = get_parameter("channel_names").as_string_array();
  auto const pixel_formats = get_parameter("pixel_formats").as_string_array();
  auto const calib_urls = get_parameter("camera_info_urls").as_string_array();
  assert(stream_names_.size() == pixel_formats.size() && stream_names_.size() == calib_urls.size());

  // check if every stream channel has been given a channel name
  if (stream_names_.size() < num_streams_) {
    num_streams_ = stream_names_.size();
  }
}

CameraAravis::~CameraAravis() = default;

void CameraAravis::setup_parameters()
{
  declare_parameter<std::string>("guid", "");
  declare_parameter<std::vector<std::string>>("channel_names",std::vector<std::string>({""}));
  declare_parameter<std::vector<std::string>>("pixel_formats", std::vector<std::string>({""}));
  declare_parameter<std::vector<std::string>>("camera_info_urls", std::vector<std::string>({""}));
}

void CameraAravis::discover_features()
{
  implemented_features_.clear();
  if (!p_device_)
    return;

  // get the root node of genicam description
  ArvGc *gc = arv_device_get_genicam(p_device_);
  if (!gc)
    return;

  std::list<ArvDomNode*> todo;
  todo.push_front((ArvDomNode*)arv_gc_get_node(gc, "Root"));

  while (!todo.empty())
  {
    // get next entry
    ArvDomNode *node = todo.front();
    todo.pop_front();
    const std::string name(arv_dom_node_get_node_name(node));

    // Do the indirection
    if (name[0] == 'p')
    {
      if (name.compare("pInvalidator") == 0)
      {
        continue;
      }
      ArvDomNode *inode = (ArvDomNode*)arv_gc_get_node(gc,
                                                       arv_dom_node_get_node_value(arv_dom_node_get_first_child(node)));
      if (inode)
        todo.push_front(inode);
      continue;
    }

    // check for implemented feature
    if (ARV_IS_GC_FEATURE_NODE(node))
    {
      ArvGcFeatureNode *fnode = ARV_GC_FEATURE_NODE(node);
      const std::string fname(arv_gc_feature_node_get_name(fnode));
      const bool usable = arv_gc_feature_node_is_available(fnode, NULL)
          && arv_gc_feature_node_is_implemented(fnode, NULL);
      if (verbose_)
      {
        RCLCPP_INFO_STREAM(get_logger(), "Feature " << fname << " is " << (usable ? "usable" : "not usable") << ".");
      }
      implemented_features_.emplace(fname, usable);
      //}
    }

    // add children in todo-list
    ArvDomNodeList *children = arv_dom_node_get_child_nodes(node);
    const uint l = arv_dom_node_list_get_length(children);
    for (uint i = 0; i < l; ++i)
    {
      todo.push_front(arv_dom_node_list_get_item(children, i));
    }
  }
}

void CameraAravis::get_num_streams()
{
  num_streams_ = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount");
  // if this return 0, try the deprecated GevStreamChannelCount in case this is an older camera
  if (num_streams_ == 0) {
    num_streams_ = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount");
  }
}

}  // end namespace camera_aravis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis::CameraAravis)
