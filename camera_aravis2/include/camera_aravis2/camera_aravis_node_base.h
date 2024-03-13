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

#ifndef CAMERA_ARAVIS2__CAMERA_ARAVIS_NODE_BASE_H_
#define CAMERA_ARAVIS2__CAMERA_ARAVIS_NODE_BASE_H_

// Std
#include <string>

// Aravis
extern "C"
{
#include "aravis-0.8/arv.h"
}

// ROS
#include <rclcpp/rclcpp.hpp>

namespace camera_aravis2
{

class CameraAravisNodeBase : public rclcpp::Node
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization constructor
     *
     * @param[in] name Node name.
     * @param[in] options Node options.
     */
    explicit CameraAravisNodeBase(const std::string& name,
                                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     *
     */
    virtual ~CameraAravisNodeBase();

    /**
     * @brief Returns true, if node is is initialized. False, otherwise.
     */
    bool is_initialized() const;

  protected:
    /**
     * @brief Set the up launch parameters.
     */
    virtual void setup_parameters();

    /**
     * @brief Discover attached camera devices found by Aravis and open device specified by guid.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool discover_and_open_camera_device();

    //--- FUNCTION DECLARATION ---//

  protected:
    /**
     * @brief Construct GUID string of given camera, using vendor name, model name and
     * device serial number.
     *
     * @return GUID in the format: <vendor_name>-<model_name>-<device_sn | device_id>.
     */
    static std::string construct_camera_guid_str(ArvCamera* p_cam);

    /**
     * @brief Handle 'control-lost' signal emitted by aravis.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handle_control_lost_signal(ArvDevice* p_device, gpointer p_user_data);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Flag indicating if node is initialized.
    bool is_initialized_;

    /// Logger object of node.
    rclcpp::Logger logger_;

    /// Pointer to Aravis device.
    ArvDevice* p_device_;

    /// Pointer to Aravis camera.
    ArvCamera* p_camera_;

    /// GUID of camera.
    std::string guid_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__CAMERA_ARAVIS_NODE_BASE_H_
