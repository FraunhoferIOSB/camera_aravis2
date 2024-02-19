/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2024 Fraunhofer IOSB and contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 ****************************************************************************/

#include "../include/camera_aravis/camera_aravis.h"

// Std
#include <chrono>

namespace camera_aravis
{

//==================================================================================================
CameraAravis::CameraAravis(const rclcpp::NodeOptions& options) :
  Node("camera_aravis", options),
  logger_(this->get_logger()),
  err_(),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_(""),
  verbose_(false)
{
    //--- setup parameters
    setup_parameters();

    //--- open camera device
    bool isSuccessful = discover_and_open_camera_device();
    if (!isSuccessful)
    {
        RCLCPP_FATAL(logger_, "Shutting down ...");

        rclcpp::shutdown();
        return;
    }

    //--- initialize stream list
    initialize_camera_streams();
}

//==================================================================================================
CameraAravis::~CameraAravis()
{

    g_object_unref(p_camera_);
}

//==================================================================================================
void CameraAravis::setup_parameters()
{
    auto guid_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    guid_desc.description = "Serial number of camera that is to be opened.";
    declare_parameter<std::string>("guid", "", guid_desc);

    auto stream_count_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    stream_count_desc.description = "Number of streams supported by the camera. Default: 1";
    declare_parameter<int>("stream_count", 1, stream_count_desc);

    auto stream_names_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    stream_names_desc.description = "[Optional] String list of names that are to be "
                                    "associated with each stream. If 'stream_count' is not set or "
                                    "set to 1, list can be empty. Otherwise, list must have the "
                                    "length of 'stream_count'. List is truncated to size of "
                                    "'stream_count' if too long.";
    declare_parameter<std::vector<std::string>>("stream_names", std::vector<std::string>({""}),
                                                stream_names_desc);

    auto pixel_formats_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    pixel_formats_desc.description = "String list of pixel formats associated with each "
                                     "stream. List must have the length of 'stream_count'. List is "
                                     "truncated to size of 'stream_count' if too long.";
    declare_parameter<std::vector<std::string>>("pixel_formats", std::vector<std::string>({""}),
                                                pixel_formats_desc);

    auto camera_info_urls_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    camera_info_urls_desc.description = "String list of urls to camera_info files associated with "
                                        "each stream. List must have the length of 'stream_count'. "
                                        "List is truncated to size of 'stream_count' if too long.";
    declare_parameter<std::vector<std::string>>("camera_info_urls", std::vector<std::string>({""}),
                                                camera_info_urls_desc);
}

//==================================================================================================
[[nodiscard]] bool CameraAravis::discover_and_open_camera_device()
{
    //--- Discover available interfaces and devices.

    arv_update_device_list();
    auto n_interfaces = arv_get_n_interfaces();
    auto n_devices    = arv_get_n_devices();

    RCLCPP_INFO(logger_, "Attached cameras:");
    RCLCPP_INFO(logger_, "\t# Interfaces: %d", n_interfaces);
    RCLCPP_INFO(logger_, "\t# Devices: %d", n_devices);
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
            p_camera_ = arv_camera_new(nullptr, err_.ref());
        }
        else
        {
            RCLCPP_INFO_STREAM(logger_, "Opening: " << guid_);
            p_camera_ = arv_camera_new(guid_.c_str(), err_.ref());
        }

        if (!p_camera_)
        {
            if (err_)
                err_.log(logger_);
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

    p_device_               = arv_camera_get_device(p_camera_);
    const char* vendor_name = arv_camera_get_vendor_name(p_camera_, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_camera_, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_camera_, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_camera_, nullptr);
    RCLCPP_INFO(logger_, "Successfully opened: %s-%s-%s",
                vendor_name, model_name, (device_sn) ? device_sn : device_id);

    return true;
}

//==================================================================================================
void CameraAravis::initialize_camera_streams()
{
    //--- get number of streams and associated names

    int num_streams   = get_parameter("stream_count").as_int();
    auto stream_names = get_parameter("stream_names").as_string_array();
    num_streams       = std::max(1, num_streams);

    //--- check if given name list corresponds to stream count

    if (static_cast<int>(stream_names.size()) != num_streams)
    {
        RCLCPP_WARN(logger_, "Size of 'stream_names' does not correspond 'num_streams'.");

        if (static_cast<int>(stream_names.size()) < num_streams)
        {
            num_streams = stream_names.size();
            RCLCPP_WARN(logger_,
                        "Only spawning %i stream(s) (length of 'steam_names').",
                        num_streams);
        }
        else
        {
            stream_names.resize(num_streams);
            RCLCPP_WARN(logger_,
                        "Truncating 'stream_names' to %i elements.",
                        static_cast<int>(stream_names.size()));
        }
    }

    //--- initialize stream list
    for (uint i = 0; i < stream_names.size(); ++i)
    {
        streams_.push_back({nullptr, CameraBufferPool::SharedPtr(), stream_names[i]});
    }
}

//==================================================================================================
void CameraAravis::discover_features()
{
    implemented_features_.clear();
    if (!p_device_)
        return;

    // get the root node of genicam description
    ArvGc* gc = arv_device_get_genicam(p_device_);
    if (!gc)
        return;

    std::list<ArvDomNode*> todo;
    todo.push_front((ArvDomNode*)arv_gc_get_node(gc, "Root"));

    while (!todo.empty())
    {
        // get next entry
        ArvDomNode* node = todo.front();
        todo.pop_front();
        const std::string name(arv_dom_node_get_node_name(node));

        // Do the indirection
        if (name[0] == 'p')
        {
            if (name.compare("pInvalidator") == 0)
            {
                continue;
            }
            ArvDomNode* inode = (ArvDomNode*)arv_gc_get_node(gc,
                                                             arv_dom_node_get_node_value(arv_dom_node_get_first_child(node)));
            if (inode)
                todo.push_front(inode);
            continue;
        }

        // check for implemented feature
        if (ARV_IS_GC_FEATURE_NODE(node))
        {
            ArvGcFeatureNode* fnode = ARV_GC_FEATURE_NODE(node);
            const std::string fname(arv_gc_feature_node_get_name(fnode));
            const bool usable = arv_gc_feature_node_is_available(fnode, NULL) && arv_gc_feature_node_is_implemented(fnode, NULL);
            if (verbose_)
            {
                RCLCPP_INFO_STREAM(logger_, "Feature " << fname << " is " << (usable ? "usable" : "not usable") << ".");
            }
            implemented_features_.emplace(fname, usable);
            //}
        }

        // add children in todo-list
        ArvDomNodeList* children = arv_dom_node_get_child_nodes(node);
        const uint l             = arv_dom_node_list_get_length(children);
        for (uint i = 0; i < l; ++i)
        {
            todo.push_front(arv_dom_node_list_get_item(children, i));
        }
    }
}

//==================================================================================================
void CameraAravis::spawn_camera_streams()
{
    // GuardedGError error;

    // for (int i = 0; i < num_streams_.size(); i++)
    // {
    //     while (is_spawning_)
    //     {
    //         Stream& stream = streams_[i];

    //         if (arv_camera_is_gv_device(p_camera_))
    //             aravis::camera::gv::select_stream_channel(p_camera_, i);

    //         stream.p_stream = aravis::camera::create_stream(p_camera_, NULL, NULL);
    //         if (stream.p_stream)
    //         {
    //             // Load up some buffers.
    //             if (arv_camera_is_gv_device(p_camera_))
    //                 aravis::camera::gv::select_stream_channel(p_camera_, i);

    //             const gint n_bytes_payload_stream_ = aravis::camera::get_payload(p_camera_);

    //             stream.p_buffer_pool.reset(new CameraBufferPool(stream.p_stream, n_bytes_payload_stream_, 10));

    //             for (int j = 0; j < stream.substreams.size(); ++j)
    //             {
    //                 // create non-aravis buffer pools for multipart part part images recycling
    //                 stream.substreams[j].p_buffer_pool.reset(new CameraBufferPool(nullptr, 0, 0));
    //                 // start substream processing threads
    //                 stream.substreams[j].buffer_thread = std::thread(&CameraAravisNodelet::substreamThreadMain, this, i, j);
    //             }

    //             if (arv_camera_is_gv_device(p_camera_))
    //                 tuneGvStream(reinterpret_cast<ArvGvStream*>(stream.p_stream));

    //             break;
    //         }
    //         else
    //         {
    //             ROS_WARN("Stream %i: Could not create image stream for %s.  Retrying...", i, guid_.c_str());
    //             ros::Duration(1.0).sleep();
    //             ros::spinOnce();
    //         }
    //     }
    // }

    // // Monitor whether anyone is subscribed to the camera stream
    // std::vector<image_transport::SubscriberStatusCallback> image_cbs_;
    // std::vector<ros::SubscriberStatusCallback> info_cbs_;

    // image_transport::SubscriberStatusCallback image_cb = [this](const image_transport::SingleSubscriberPublisher& ssp)
    // { this->rosConnectCallback(); };
    // ros::SubscriberStatusCallback info_cb = [this](const ros::SingleSubscriberPublisher& ssp)
    // { this->rosConnectCallback(); };

    // for (int i = 0; i < streams_.size(); i++)
    // {
    //     for (int j = 0; j < streams_[i].substreams.size(); ++j)
    //     {
    //         image_transport::ImageTransport* p_transport;
    //         const Substream& sub = streams_[i].substreams[j];

    //         // Set up image_raw
    //         std::string topic_name = this->getName();
    //         p_transport            = new image_transport::ImageTransport(pnh);
    //         if (streams_.size() != 1 || streams_[i].substreams.size() != 1 || !sub.name.empty())
    //         {
    //             topic_name += "/" + sub.name;
    //         }

    //         streams_[i].substreams[j].cam_pub = p_transport->advertiseCamera(
    //           ros::names::remap(topic_name + "/image_raw"),
    //           1, image_cb, image_cb, info_cb, info_cb);
    //     }
    // }

    // // Connect signals with callbacks.
    // for (int i = 0; i < streams_.size(); i++)
    // {
    //     StreamIdData* data = new StreamIdData();
    //     data->can          = this;
    //     data->stream_id    = i;
    //     g_signal_connect(streams_[i].p_stream, "new-buffer", (GCallback)CameraAravisNodelet::newBufferReadyCallback, data);
    // }
    // g_signal_connect(p_device_, "control-lost", (GCallback)CameraAravisNodelet::controlLostCallback, this);

    // for (int i = 0; i < streams_.size(); i++)
    // {
    //     arv_stream_set_emit_signals(streams_[i].p_stream, TRUE);
    // }

    // // any substream of any stream enabled?
    // if (std::any_of(streams_.begin(), streams_.end(),
    //                 [](const Stream& src)
    //                 {
    //                     return std::any_of(src.substreams.begin(), src.substreams.end(),
    //                                        [](const Substream& sub)
    //                                        {
    //                                            return sub.cam_pub.getNumSubscribers() > 0;
    //                                        });
    //                 }))
    // {
    //     aravis::camera::start_acquisition(p_camera_);
    // }
}

} // end namespace camera_aravis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis::CameraAravis)
