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

#include "../include/camera_aravis/genicam_xml_exporter.h"

// Std
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

// camera_aravis2
#include "../include/camera_aravis/common.h"
#include "../include/camera_aravis/error.h"

namespace camera_aravis2
{

//==================================================================================================
GenICamXmlExporter::GenICamXmlExporter(const rclcpp::NodeOptions& options) :
  Node("genicam_xml_exporter", options),
  logger_(this->get_logger()),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_(""),
  xml_file_path_("")
{
    //--- setup parameters
    setup_parameters();

    //--- open camera device
    ASSERT_SUCCESS(discover_and_open_camera_device());

    std::string camera_guid_str = construct_camera_guid_str(p_camera_);
    RCLCPP_INFO(logger_, "Successfully Opened: %s", camera_guid_str.c_str());

    //--- export XML data to file
    ASSERT_SUCCESS(export_xml_data_to_file());

    RCLCPP_INFO(logger_, "Written GenICam XML to file: %s",
                std::filesystem::canonical(xml_file_path_).c_str());
}

//==================================================================================================
GenICamXmlExporter::~GenICamXmlExporter()
{
    RCLCPP_INFO(logger_, "Shutting down ...");

    // Guarded error object
    GuardedGError err;

    //--- stop acquisition
    if (p_device_)
    {
        arv_device_execute_command(p_device_, "AcquisitionStop", err.ref());
        CHECK_GERROR(err, logger_);
    }

    //--- unref pointers
    g_object_unref(p_camera_);
}

//==================================================================================================
void GenICamXmlExporter::setup_parameters()
{
    auto guid_desc = rcl_interfaces::msg::ParameterDescriptor{};
    guid_desc.description =
      "Serial number of camera that is to be opened.";
    declare_parameter<std::string>("guid", "", guid_desc);

    auto xmlFile_desc = rcl_interfaces::msg::ParameterDescriptor{};
    xmlFile_desc.description =
      "Path to XML output file. If omitted, the XML data will be written into file with GUID as "
      "file name.";
    declare_parameter<std::string>("xml_file", "", xmlFile_desc);
}

//==================================================================================================
[[nodiscard]] bool GenICamXmlExporter::discover_and_open_camera_device()
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

    //--- check if GEV Device.
    // TODO: Remove, when USB Devices are supported.
    if (!arv_camera_is_gv_device(p_camera_))
    {
        RCLCPP_FATAL(logger_, "Camera is no GEV Device.");
        RCLCPP_FATAL(logger_, "USB3 Devices are currently not supported.");
        RCLCPP_FATAL(logger_,
                     "Help Wanted: https://github.com/FraunhoferIOSB/camera_aravis2/issues/14");
        return false;
    }

    //--- get device pointer from camera
    p_device_ = arv_camera_get_device(p_camera_);

    return true;
}

//==================================================================================================
[[nodiscard]] bool GenICamXmlExporter::export_xml_data_to_file()
{
    /// Path string of output xml file.
    std::string xml_file_str = get_parameter("xml_file").as_string();

    if (xml_file_str.empty())
    {
        std::string tmpFileName = (guid_.empty())
                                    ? construct_camera_guid_str(p_camera_)
                                    : guid_;

        // replace whitespaces in url (coming from GUID) with '_'
        std::replace(tmpFileName.begin(), tmpFileName.end(), ' ', '_');

        xml_file_path_ = std::filesystem::path(tmpFileName + ".xml");
    }
    else
    {
        xml_file_path_ = std::filesystem::path(xml_file_str);
    }

    //--- make path absolute
    xml_file_path_ = std::filesystem::absolute(xml_file_path_);

    //--- print warning if file already exists
    if (std::filesystem::exists(xml_file_path_))
        RCLCPP_WARN(logger_,
                    "Output file already exists and will be overwritten. Path: %s",
                    std::filesystem::canonical(xml_file_path_).c_str());

    //--- make parent directory if not existing
    if (!xml_file_path_.parent_path().empty())
        std::filesystem::create_directories(xml_file_path_.parent_path());

    //--- extract and save XML
    size_t xml_size        = 0;
    const char* p_xml_data = arv_device_get_genicam_xml(p_device_, &xml_size);
    std::ofstream fout;
    fout.open(xml_file_path_.c_str(), std::ios::binary | std::ios::out);
    fout.write(p_xml_data, xml_size);
    fout.close();

    return true;
}

//==================================================================================================
inline std::string GenICamXmlExporter::construct_camera_guid_str(ArvCamera* p_cam)
{
    const char* vendor_name = arv_camera_get_vendor_name(p_cam, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_cam, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_cam, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_cam, nullptr);

    return (std::string(vendor_name) + "-" +
            std::string(model_name) + "-" +
            std::string((device_sn) ? device_sn : device_id));
}

}  // end namespace camera_aravis2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis2::GenICamXmlExporter)
