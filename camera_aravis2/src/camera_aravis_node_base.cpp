// Copyright (c) 2024 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "../include/camera_aravis2/camera_aravis_node_base.h"

// camera_aravis2
#include "../include/camera_aravis2/common.h"
#include "../include/camera_aravis2/error.h"

namespace camera_aravis2
{

//==================================================================================================
CameraAravisNodeBase::CameraAravisNodeBase(const std::string& name,
                                           const rclcpp::NodeOptions& options) :
  Node(name, options),
  is_initialized_(false),
  logger_(this->get_logger()),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_("")
{
}

//==================================================================================================
CameraAravisNodeBase::~CameraAravisNodeBase()
{
    //--- unreference camera pointer
    if (p_camera_)
        g_object_unref(p_camera_);

    RCLCPP_INFO(logger_, "Node has shut down.");
}

//==================================================================================================
bool CameraAravisNodeBase::is_initialized() const
{
    return (is_initialized_);
}

//==================================================================================================
void CameraAravisNodeBase::setup_parameters()
{
    auto guid_desc = rcl_interfaces::msg::ParameterDescriptor{};
    guid_desc.description =
      "Serial number of camera that is to be opened.";
    declare_parameter<std::string>("guid", "", guid_desc);
}

//==================================================================================================
[[nodiscard]] bool CameraAravisNodeBase::discover_and_open_camera_device()
{
    // Guarded error object
    GuardedGError err;

    //--- Discover available interfaces and devices.

    arv_update_device_list();
    auto n_interfaces = arv_get_n_interfaces();
    auto n_devices    = arv_get_n_devices();

    RCLCPP_INFO(logger_, "Attached cameras:");
    RCLCPP_INFO(logger_, "\tNum. Interfaces: %d", n_interfaces);
    RCLCPP_INFO(logger_, "\tNum. Devices: %d", n_devices);
    for (uint i = 0; i < n_devices; i++)
        RCLCPP_INFO(logger_, "\tDevice %d: %s", i, arv_get_device_id(i));

    if (n_devices == 0)
    {
        RCLCPP_FATAL(logger_, "No cameras detected.");
        return false;
    }

    //--- connect to camera specified by guid parameter
    guid_ = get_parameter("guid").as_string();

    const int MAX_RETRIES = 10;
    int tryCount          = 1;
    while (!p_camera_ && tryCount <= MAX_RETRIES)
    {
        if (guid_ == "")
        {
            RCLCPP_WARN(logger_, "No guid specified.");
            RCLCPP_INFO(logger_, "Opening: (any)");
            p_camera_ = arv_camera_new(nullptr, err.ref());
        }
        else
        {
            RCLCPP_INFO(logger_, "Opening: %s ", guid_.c_str());
            p_camera_ = arv_camera_new(guid_.c_str(), err.ref());
        }

        if (!p_camera_)
        {
            CHECK_GERROR(err, logger_);
            RCLCPP_WARN(logger_, "Unable to open camera. Retrying (%i/%i) ...",
                        tryCount, MAX_RETRIES);
            rclcpp::sleep_for(std::chrono::seconds(1));
            tryCount++;
        }
    }

    if (!p_camera_)
    {
        RCLCPP_FATAL(logger_, "Failed to open any camera.");
        return false;
    }

    //--- get device pointer from camera
    p_device_ = arv_camera_get_device(p_camera_);

    //--- connect control-lost signal
    g_signal_connect(p_device_, "control-lost",
                     (GCallback)CameraAravisNodeBase::handle_control_lost_signal, this);

    return true;
}

//==================================================================================================
std::string CameraAravisNodeBase::construct_camera_guid_str(ArvCamera* p_cam)
{
    const char* vendor_name = arv_camera_get_vendor_name(p_cam, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_cam, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_cam, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_cam, nullptr);

    return (std::string(vendor_name) + "-" +
            std::string(model_name) + "-" +
            std::string((device_sn) ? device_sn : device_id));
}

//==================================================================================================
void CameraAravisNodeBase::handle_control_lost_signal(ArvDevice* p_device, gpointer p_user_data)
{
    RCL_UNUSED(p_device);

    CameraAravisNodeBase* p_ca_instance = reinterpret_cast<CameraAravisNodeBase*>(p_user_data);

    if (!p_ca_instance)
        return;

    RCLCPP_FATAL(p_ca_instance->logger_, "Control to aravis device lost.");
    RCLCPP_FATAL(p_ca_instance->logger_, "\tGUID: %s", p_ca_instance->guid_.c_str());

    rclcpp::shutdown();
}

}  // end namespace camera_aravis2
