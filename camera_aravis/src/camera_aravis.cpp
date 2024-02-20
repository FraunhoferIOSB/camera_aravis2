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
#include <iostream>
#include <thread>

// Macro to assert success of given function
#define ASSERT_SUCCESS(fn)  \
    if (!fn)                \
    {                       \
        rclcpp::shutdown(); \
        return;             \
    }

namespace camera_aravis
{

//==================================================================================================
CameraAravis::CameraAravis(const rclcpp::NodeOptions& options) :
  Node("camera_aravis", options),
  logger_(this->get_logger()),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_(""),
  is_spawning_(false),
  verbose_(false)
{
    //--- setup parameters
    setup_parameters();

    //--- open camera device
    ASSERT_SUCCESS(discover_and_open_camera_device());

    //--- initialize stream list
    ASSERT_SUCCESS(initialize_camera_streams());

    //--- spawn camera stream in thread, so that initialization is not blocked
    is_spawning_         = true;
    spawn_stream_thread_ = std::thread(&CameraAravis::spawn_camera_streams, this);
}

//==================================================================================================
CameraAravis::~CameraAravis()
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

    //--- stop emitting signals for streams
    for (uint i = 0; i < streams_.size(); i++)
        if (streams_[i].p_arv_stream)
            arv_stream_set_emit_signals(streams_[i].p_arv_stream, FALSE);

    //--- join spawning thread
    is_spawning_ = false;
    if (spawn_stream_thread_.joinable())
        spawn_stream_thread_.join();

    //--- print stream statistics
    printStreamStatistics();

    //--- unref pointers
    for (uint i = 0; i < streams_.size(); i++)
    {
        if (streams_[i].p_arv_stream)
            g_object_unref(streams_[i].p_arv_stream);
    }
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
        RCLCPP_FATAL(logger_, "Help Wanted: https://github.com/FraunhoferIOSB/camera_aravis2/issues/14");
        return false;
    }

    //--- connect control-lost signal
    g_signal_connect(p_device_, "control-lost", (GCallback)CameraAravis::handleControlLost, this);

    return true;
}

//==================================================================================================
bool CameraAravis::initialize_camera_streams()
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

    //--- check if at least one stream was initialized
    if (streams_.empty())
    {
        RCLCPP_FATAL(logger_, "Something went wrong in the initialization of the camera streams.");
        return false;
    }

    return true;
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
    GuardedGError err;

    // Number of opened streams
    int num_opened_streams = 0;

    for (uint i = 0; i < streams_.size(); i++)
    {
        RCLCPP_INFO(logger_, "Spawning camera stream with ID %i", i);

        Stream& stream = streams_[i];

        const int MAX_RETRIES = 60;
        int tryCount          = 1;
        while (is_spawning_ && tryCount <= MAX_RETRIES)
        {

            arv_camera_gv_select_stream_channel(p_camera_, i, err.ref());
            stream.p_arv_stream = arv_camera_create_stream(p_camera_, nullptr, nullptr, err.ref());
            CHECK_GERROR(err, logger_);

            if (stream.p_arv_stream)
            {
                //--- Initialize buffers

                // stream payload size in bytes
                const auto STREAM_PAYLOAD_SIZE = arv_camera_get_payload(p_camera_, err.ref());
                CHECK_GERROR(err, logger_);

                // TODO: launch parameter for number of preallocated buffers
                stream.p_buffer_pool.reset(
                  new CameraBufferPool(logger_, stream.p_arv_stream,
                                       static_cast<guint>(STREAM_PAYLOAD_SIZE), 10));

                tuneGvStream(reinterpret_cast<ArvGvStream*>(stream.p_arv_stream));

                num_opened_streams++;
                break;
            }
            else
            {
                RCLCPP_WARN(logger_, "%s: Could not create image stream with ID %i. "
                                     "Retrying (%i/%i) ...",
                            guid_.c_str(), i, tryCount, MAX_RETRIES);
                rclcpp::sleep_for(std::chrono::seconds(1));
                tryCount++;
            }
        }

        //--- check if stream could be established
        if (!stream.p_arv_stream)
            RCLCPP_ERROR(logger_, "%s: Could not create image stream with ID %i.",
                         guid_.c_str(), i);
    }
    is_spawning_ = false;

    //--- if no streams are opened, shut down
    if (num_opened_streams == 0)
    {
        RCLCPP_FATAL(logger_, "camera_aravis failed to open streams for camera %s.",
                     guid_.c_str());
        ASSERT_SUCCESS(false);
    }

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

    //--- Connect signals with callbacks and activate emission of signals
    for (Stream stream : streams_)
    {
        if (!stream.p_arv_stream)
            continue;
        // StreamIdData* data = new StreamIdData();
        // data->can          = this;
        // data->stream_id    = i;

        g_signal_connect(stream.p_arv_stream, "new-buffer",
                         (GCallback)CameraAravis::handleNewBufferReady, nullptr);

        arv_stream_set_emit_signals(stream.p_arv_stream, TRUE);
    }

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
    arv_camera_start_acquisition(p_camera_, err.ref());
    CHECK_GERROR(err, logger_);
    // }

    //--- print final output message
    p_device_               = arv_camera_get_device(p_camera_);
    const char* vendor_name = arv_camera_get_vendor_name(p_camera_, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_camera_, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_camera_, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_camera_, nullptr);

    RCLCPP_INFO(logger_, "Done initializing camera_aravis.");
    RCLCPP_INFO(logger_, "\tCamera: %s-%s-%s",
                vendor_name, model_name, (device_sn) ? device_sn : device_id);
    RCLCPP_INFO(logger_, "\tNum. Streams: (%i / %i)",
                num_opened_streams, static_cast<int>(streams_.size()));
}

//==================================================================================================
void CameraAravis::tuneGvStream(ArvGvStream* p_stream) const
{
    if (!p_stream)
        return;

    gboolean b_auto_buffer               = FALSE;
    gboolean b_packet_resend             = TRUE;
    unsigned int timeout_packet          = 40; // milliseconds
    unsigned int timeout_frame_retention = 200;

    if (!ARV_IS_GV_STREAM(p_stream))
    {
        RCLCPP_ERROR(logger_, "Stream is not a GV_STREAM");
        return;
    }

    if (b_auto_buffer)
        g_object_set(p_stream, "socket-buffer",
                     ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                     "socket-buffer-size", 0,
                     NULL);
    if (!b_packet_resend)
        g_object_set(p_stream, "packet-resend",
                     b_packet_resend
                       ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                       : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                     NULL);
    g_object_set(p_stream, "packet-timeout",
                 timeout_packet * 1000,
                 "frame-retention", timeout_frame_retention * 1000,
                 NULL);
}

//==================================================================================================
void CameraAravis::printStreamStatistics() const
{
    for (uint i = 0; i < streams_.size(); i++)
    {
        const Stream& STREAM = streams_[i];

        if (!STREAM.p_arv_stream)
            continue;

        guint64 n_completed_buffers = 0;
        guint64 n_failures          = 0;
        guint64 n_underruns         = 0;
        arv_stream_get_statistics(STREAM.p_arv_stream,
                                  &n_completed_buffers, &n_failures, &n_underruns);

        RCLCPP_INFO(logger_, "Statistics for stream %i:", i);
        RCLCPP_INFO(logger_, "\tCompleted buffers = %lli", (unsigned long long)n_completed_buffers);
        RCLCPP_INFO(logger_, "\tFailures          = %lli", (unsigned long long)n_failures);
        RCLCPP_INFO(logger_, "\tUnderruns         = %lli", (unsigned long long)n_underruns);

        if (arv_camera_is_gv_device(p_camera_))
        {
            guint64 n_resent;
            guint64 n_missing;

            arv_gv_stream_get_statistics(reinterpret_cast<ArvGvStream*>(STREAM.p_arv_stream),
                                         &n_resent, &n_missing);
            RCLCPP_INFO(logger_, "\tResent buffers    = %lli", (unsigned long long)n_resent);
            RCLCPP_INFO(logger_, "\tMissing           = %lli", (unsigned long long)n_missing);
        }
    }
}

//==================================================================================================
void CameraAravis::handleControlLost(ArvDevice* p_device, gpointer p_user_data)
{
    RCL_UNUSED(p_device);

    CameraAravis* p_ca_instance = (CameraAravis*)p_user_data;

    if (!p_ca_instance)
        return;

    RCLCPP_FATAL(p_ca_instance->logger_, "Control to aravis device lost.");
    RCLCPP_FATAL(p_ca_instance->logger_, "\tGUID: %s", p_ca_instance->guid_.c_str());

    rclcpp::shutdown();
}

//==================================================================================================
void CameraAravis::handleNewBufferReady(ArvStream* p_stream, gpointer p_user_data)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

} // end namespace camera_aravis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis::CameraAravis)
