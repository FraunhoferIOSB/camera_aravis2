// Copyright (c) 2024 Fraunhofer IOSB and contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef CAMERA_ARAVIS__GENICAM_XML_EXPORTER_H_
#define CAMERA_ARAVIS__GENICAM_XML_EXPORTER_H_

// Std
#include <filesystem>
#include <string>

// Aravis
extern "C"
{
#include "aravis-0.8/arv.h"
}

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

namespace camera_aravis2
{

class GenICamXmlExporter : public rclcpp::Node
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization constructor
     *
     * @param[in] options Node options.
     */
    explicit GenICamXmlExporter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     *
     */
    ~GenICamXmlExporter() override;

  private:
    /**
     * @brief Set the up launch parameters.
     */
    void setup_parameters();

    /**
     * @brief Discover attached camera devices found by Aravis and open device specified by guid.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool discover_and_open_camera_device();

    /**
     * @brief Export XML data from camera and write to file.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool export_xml_data_to_file();

    //--- FUNCTION DECLARATION ---//

  protected:
    /**
     * @brief Construct GUID string of given camera, using vendor name, model name and
     * device serial number.
     *
     * @return GUID in the format: <vendor_name>-<model_name>-<device_sn | device_id>.
     */
    static inline std::string construct_camera_guid_str(ArvCamera* p_cam);

    //--- MEMBER DECLARATION ---//

  private:
    /// Logger object of node.
    rclcpp::Logger logger_;

    /// Pointer to Aravis device.
    ArvDevice* p_device_;

    /// Pointer to Aravis camera.
    ArvCamera* p_camera_;

    /// GUID of camera.
    std::string guid_;

    /// Path of output xml file.
    std::filesystem::path xml_file_path_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS__GENICAM_XML_EXPORTER_H_
