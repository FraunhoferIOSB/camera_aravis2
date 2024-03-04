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

// camera_aravis
#include "../include/camera_aravis/conversion_utils.h"

// #define MULTITHREAD_INSPECTION

// Macro to assert success of given function
#define ASSERT_SUCCESS(fn) \
    if (!fn)               \
    {                      \
        return;            \
    }
// Macro to assert success of given function and shut down if not successful
#define ASSERT_SUCCESS_AND_SHUTDOWN(fn) \
    if (!fn)                            \
    {                                   \
        rclcpp::shutdown();             \
        return;                         \
    }

// Conversions from integers to Arv types.
static const char* arvBufferStatusFromInt[] =
  {"ARV_BUFFER_STATUS_SUCCESS", "ARV_BUFFER_STATUS_CLEARED",
   "ARV_BUFFER_STATUS_TIMEOUT", "ARV_BUFFER_STATUS_MISSING_PACKETS",
   "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
   "ARV_BUFFER_STATUS_FILLING", "ARV_BUFFER_STATUS_ABORTED"};

#ifdef MULTITHREAD_INSPECTION
static std::chrono::time_point<std::chrono::system_clock> start;
static int count = 0;
#endif

namespace camera_aravis
{

//==================================================================================================
CameraAravis::CameraAravis(const rclcpp::NodeOptions& options) :
  Node("camera_aravis", options),
  is_initialized_(false),
  logger_(this->get_logger()),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_(""),
  is_spawning_(false),
  use_ptp_timestamp_(false),
  verbose_(false)
{
#ifdef MULTITHREAD_INSPECTION
    RCLCPP_INFO_STREAM(logger_, ">>>>>>>>>> Main thread ID: " << std::this_thread::get_id());
    RCLCPP_INFO_STREAM(logger_, ">>>>>>>>>> CameraAravis*: " << this);
#endif

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

    //--- join buffer threads
    for (uint i = 0; i < streams_.size(); i++)
    {
        Stream& stream = streams_[i];

        stream.is_buffer_processed = false;

        //--- push empty object into queue to do final loop
        stream.buffer_queue.push(std::make_tuple(nullptr, nullptr));

        if (stream.buffer_processing_thread.joinable())
            stream.buffer_processing_thread.join();
    }

    //--- print stream statistics
    print_stream_statistics();

    //--- unref pointers
    for (uint i = 0; i < streams_.size(); i++)
    {
        if (streams_[i].p_arv_stream)
            g_object_unref(streams_[i].p_arv_stream);
    }
    g_object_unref(p_camera_);
}

//==================================================================================================
bool CameraAravis::is_spawning_or_initialized() const
{

    return (is_spawning_ || is_initialized_);
}

//==================================================================================================
void CameraAravis::setup_parameters()
{
    auto guid_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    guid_desc.description = "Serial number of camera that is to be opened.";
    declare_parameter<std::string>("guid", "", guid_desc);

    auto stream_names_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    stream_names_desc.description = "Optional string list of names that are to be "
                                    "associated with each stream. If multiple steams are available "
                                    "but no names are given, each stream will get given a name "
                                    "based on its ID, starting with 0.";
    declare_parameter<std::vector<std::string>>("stream_names", std::vector<std::string>({}),
                                                stream_names_desc);

    auto pixel_formats_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    pixel_formats_desc.description = "String list of pixel formats associated with each "
                                     "stream. List must have the same length as 'camera_info_urls'. "
                                     "If both lists have different lengths, they are truncated to "
                                     "the size of the shorter one.";
    declare_parameter<std::vector<std::string>>("pixel_formats", std::vector<std::string>({}),
                                                pixel_formats_desc);

    auto camera_info_urls_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    camera_info_urls_desc.description = "String list of urls to camera_info files associated with "
                                        "each stream. List must have the same length as "
                                        "'pixel_formats'. If both lists have different lengths, "
                                        "they are truncated to the size of the shorter one.";
    declare_parameter<std::vector<std::string>>("camera_info_urls", std::vector<std::string>({}),
                                                camera_info_urls_desc);

    auto frame_id_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    frame_id_desc.description = "Frame ID that is to be associated with the sensor and, in turn, "
                                "with the image data. If multiple streams are supported by the "
                                "camera, the given ID serves as a base string to which the "
                                "stream name is appended, together with '_' as separator. If no "
                                "frame ID is specified, the name of the node will be used.";
    declare_parameter<std::string>("frame_id", "", frame_id_desc);
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

    //--- get device pointer from camera
    p_device_ = arv_camera_get_device(p_camera_);

    //--- connect control-lost signal
    g_signal_connect(p_device_, "control-lost", (GCallback)CameraAravis::handle_control_lost_signal, this);

    return true;
}

//==================================================================================================
bool CameraAravis::initialize_camera_streams()
{
    //--- get number of streams and associated names

    int num_streams       = discover_stream_number();
    auto stream_names     = get_parameter("stream_names").as_string_array();
    auto pixel_formats    = get_parameter("pixel_formats").as_string_array();
    auto camera_info_urls = get_parameter("camera_info_urls").as_string_array();
    auto base_frame_id    = get_parameter("frame_id").as_string();

    //--- check that at least one pixel format and one camera_info_url is specified
    if (pixel_formats.empty())
    {
        RCLCPP_FATAL(logger_, "At least one 'pixel_format' needs to be specified.");
        return false;
    }
    if (camera_info_urls.empty())
    {
        RCLCPP_FATAL(logger_, "At least one 'camera_info_url' needs to be specified.");
        return false;
    }

    //--- check if same number of pixel_formats and camera_info_urls are provided
    if (pixel_formats.size() != camera_info_urls.size())
    {
        RCLCPP_WARN(logger_,
                    "Different number of 'pixel_formats' and 'camera_info_urls' specified.");

        if (pixel_formats.size() < camera_info_urls.size())
        {
            camera_info_urls.resize(pixel_formats.size());
            RCLCPP_WARN(logger_,
                        "Truncating 'camera_info_urls' to first %i elements.",
                        static_cast<int>(pixel_formats.size()));
        }
        else
        {
            pixel_formats.resize(camera_info_urls.size());
            RCLCPP_WARN(logger_,
                        "Truncating 'pixel_formats' to first %i elements.",
                        static_cast<int>(camera_info_urls.size()));
        }
    }

    //--- check if number pixel_formats corresponds to available number of streams
    if (static_cast<int>(pixel_formats.size()) != num_streams)
    {
        if (static_cast<int>(pixel_formats.size()) > num_streams)
        {
            RCLCPP_WARN(logger_,
                        "Insufficient number of streams supported by camera.");
            RCLCPP_WARN(logger_,
                        "Truncating 'pixel_formats' and 'camera_info_urls' to first %i elements.",
                        num_streams);
        }
        else
        {
            num_streams = static_cast<int>(pixel_formats.size());
        }
    }

    //--- check if frame_id is empty
    if (base_frame_id.empty())
    {
        base_frame_id = this->get_fully_qualified_name();
        base_frame_id.replace(0, 1, ""); // remove first '/'
        std::replace(base_frame_id.begin(), base_frame_id.end(), '/', '_');
    }

    //--- initialize stream list
    streams_ = std::vector<Stream>(num_streams);
    for (uint i = 0; i < streams_.size(); ++i)
    {
        Stream& stream = streams_[i];

        //--- if no or insufficient stream names are specified, use ID as name
        stream.name            = (i < stream_names.size())
                                   ? stream_names[i]
                                   : ("stream" + std::to_string(i));
        stream.sensor.frame_id = (!stream_names.empty() || num_streams > 1)
                                   ? base_frame_id + "_" + stream.name
                                   : base_frame_id;

        //--- get pixel format and set conversion function
        stream.sensor.pixel_format = pixel_formats[i];
        const auto sensor_iter     = CONVERSIONS_DICTIONARY.find(stream.sensor.pixel_format);
        if (sensor_iter != CONVERSIONS_DICTIONARY.end())
            stream.cvt_pixel_format = sensor_iter->second;
        else
            RCLCPP_WARN(logger_, "There is no known conversion from %s to a usual ROS image "
                                 "encoding. Likely you need to implement one.",
                        stream.sensor.pixel_format.c_str());

        //--- setup image topic and create publisher
        std::string topic_name = this->get_name();
        // p_transport            = new image_transport::ImageTransport(pnh);
        if (!stream_names.empty() || num_streams > 1) // if more than one stream available, add stream name
            topic_name += "/" + stream.name;
        topic_name += "/image_raw";

        stream.camera_pub = image_transport::create_camera_publisher(this, topic_name);
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
int CameraAravis::discover_stream_number()
{
    int num_streams = 0;

    if (p_device_)
    {
        num_streams = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount",
                                                           nullptr);

        if (num_streams == 0 && arv_camera_is_gv_device(p_camera_))
        {
            num_streams = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount",
                                                               nullptr);
        }
    }

    if (num_streams == 0 || !p_device_)
    {
        num_streams = 1;
        RCLCPP_INFO(logger_, "Unable to automatically detect number of supported stream channels. "
                             "Setting num_streams = %i.",
                    num_streams);
    }
    else
    {
        RCLCPP_INFO(logger_, "Number of supported stream channels: %i", num_streams);
    }

    return num_streams;
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
        Stream& stream = streams_[i];

        RCLCPP_INFO(logger_, "Spawning camera stream with ID %i (%s)", i,
                    stream.sensor.frame_id.c_str());

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

                stream.is_buffer_processed = true;
                stream.buffer_processing_thread =
                  std::thread(&CameraAravis::process_stream_buffer, this, i);

                tune_gv_stream(reinterpret_cast<ArvGvStream*>(stream.p_arv_stream));

                num_opened_streams++;
                break;
            }
            else
            {
                RCLCPP_WARN(logger_, "%s: Could not create image stream with ID %i (%s). "
                                     "Retrying (%i/%i) ...",
                            guid_.c_str(), i, stream.sensor.frame_id.c_str(),
                            tryCount, MAX_RETRIES);
                rclcpp::sleep_for(std::chrono::seconds(1));
                tryCount++;
            }
        }

        //--- check if stream could be established
        if (!stream.p_arv_stream)
            RCLCPP_ERROR(logger_, "%s: Could not create image stream with ID %i (%s).",
                         guid_.c_str(), i, stream.sensor.frame_id.c_str());
    }
    is_spawning_ = false;

    //--- if no streams are opened, shut down
    if (num_opened_streams == 0)
    {
        RCLCPP_FATAL(logger_, "camera_aravis failed to open streams for camera %s.",
                     guid_.c_str());
        ASSERT_SUCCESS(false);
    }

    //--- Connect signals with callbacks and activate emission of signals
    for (uint i = 0; i < streams_.size(); ++i)
    {
        const Stream& STREAM = streams_[i];

        if (!STREAM.p_arv_stream)
            continue;

        new_buffer_cb_data_ptrs.push_back(
          std::make_shared<std::tuple<CameraAravis*, uint>>(
            std::make_tuple(this, i)));

        g_signal_connect(STREAM.p_arv_stream, "new-buffer",
                         (GCallback)CameraAravis::handle_new_buffer_signal,
                         new_buffer_cb_data_ptrs.back().get());

        arv_stream_set_emit_signals(STREAM.p_arv_stream, TRUE);
    }

    // TODO: Only start acquisition when topic is subscribed
    arv_camera_start_acquisition(p_camera_, err.ref());
    CHECK_GERROR(err, logger_);

    //--- print final output message
    const char* vendor_name = arv_camera_get_vendor_name(p_camera_, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_camera_, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_camera_, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_camera_, nullptr);

    RCLCPP_INFO(logger_, "Done initializing camera_aravis.");
    RCLCPP_INFO(logger_, "\tCamera: %s-%s-%s",
                vendor_name, model_name, (device_sn) ? device_sn : device_id);
    RCLCPP_INFO(logger_, "\tNum. Streams: (%i / %i)",
                num_opened_streams, static_cast<int>(streams_.size()));

    this->is_initialized_ = true;
}

//==================================================================================================
void CameraAravis::tune_gv_stream(ArvGvStream* p_stream) const
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
void CameraAravis::process_stream_buffer(const uint stream_id)
{
    using namespace std::chrono_literals;

    Stream& stream = streams_[stream_id];

    RCLCPP_INFO(logger_, "Started processing thread for stream %i (%s)", stream_id,
                stream.sensor.frame_id.c_str());

#ifdef MULTITHREAD_INSPECTION
    RCLCPP_INFO_STREAM(logger_, ">>>>>>>>>> Processing thread ID for stream "
                                  << stream_id
                                  << ": " << std::this_thread::get_id());
#endif

    while (stream.is_buffer_processed)
    {
        //--- pop buffer pointer from queue
        std::tuple<ArvBuffer*, sensor_msgs::msg::Image::SharedPtr> buffer_img_tuple;
        bool is_pop_successful = stream.buffer_queue.pop(buffer_img_tuple);
        if (!is_pop_successful)
            continue;

        //--- take ownership of pointers
        ArvBuffer* p_arv_buffer                      = std::get<0>(buffer_img_tuple);
        sensor_msgs::msg::Image::SharedPtr p_img_msg = std::get<1>(buffer_img_tuple);
        if (!p_arv_buffer || !p_img_msg)
            continue;

        //--- set meta data of image message
        set_image_msg_metadata(p_img_msg, p_arv_buffer, stream.sensor.frame_id);

        //--- convert to ros format
        if (stream.cvt_pixel_format)
        {
            sensor_msgs::msg::Image::SharedPtr p_cvt_img_msg =
              stream.p_buffer_pool->getRecyclableImg();
            stream.cvt_pixel_format(p_img_msg, p_cvt_img_msg);
            p_img_msg = p_cvt_img_msg;
        }

        stream.camera_pub.publish(p_img_msg, std::make_shared<sensor_msgs::msg::CameraInfo>());

#ifdef MULTITHREAD_INSPECTION
        if (count == 0)
            start = std::chrono::system_clock::now();
        count++;
        auto now                                      = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - start;

        std::cout << "Hz " << (static_cast<double>(count) / elapsed_seconds.count()) << std::endl;

        RCLCPP_INFO_STREAM(logger_, ">>>>>>>>>> TIME: "
                                      << rclcpp::Clock{}.now().seconds());
#endif
    }

    RCLCPP_INFO(logger_, "Finished processing thread for stream %i (%s)", stream_id,
                stream.sensor.frame_id.c_str());
}

//==================================================================================================
void CameraAravis::set_image_msg_metadata(sensor_msgs::msg::Image::SharedPtr& p_img_msg,
                                          ArvBuffer* p_buffer,
                                          const std::string frame_id) const
{
    //--- fill header data

    p_img_msg->header.stamp    = rclcpp::Time(use_ptp_timestamp_
                                                ? arv_buffer_get_timestamp(p_buffer)
                                                : arv_buffer_get_system_timestamp(p_buffer));
    p_img_msg->header.frame_id = frame_id;

    //--- fill payload

    // TODO: Support ROI
    gint x, y, width, height;
    arv_buffer_get_image_region(p_buffer, &x, &y, &width, &height);
    p_img_msg->width  = width;
    p_img_msg->height = height;

    // TODO: Support different encodings
    p_img_msg->encoding = "rgb8";
    p_img_msg->step     = (width * 24) / 8; // TODO: Bits per pixel.
}

//==================================================================================================
void CameraAravis::print_stream_statistics() const
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

        RCLCPP_INFO(logger_, "Statistics for stream %i (%s):", i, STREAM.sensor.frame_id.c_str());
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
void CameraAravis::handle_control_lost_signal(ArvDevice* p_device, gpointer p_user_data)
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
void CameraAravis::handle_new_buffer_signal(ArvStream* p_stream, gpointer p_user_data)
{

    ///--- get data tuples from user data
    std::tuple<CameraAravis*, uint>* p_data_tuple = (std::tuple<CameraAravis*, uint>*)p_user_data;
    CameraAravis* p_ca_instance                   = std::get<0>(*p_data_tuple);
    uint stream_id                                = std::get<1>(*p_data_tuple);

    Stream& stream = p_ca_instance->streams_[stream_id];

    ///--- get aravis buffer
    ArvBuffer* p_arv_buffer = arv_stream_try_pop_buffer(p_stream);

    //--- check if enough buffers are left, if not allocate one more for the next image
    gint n_available_buffers;
    arv_stream_get_n_buffers(p_stream, &n_available_buffers, NULL);
    if (n_available_buffers == 0)
        stream.p_buffer_pool->allocateBuffers(1);

    if (p_arv_buffer == nullptr)
        return;

    bool buffer_success = arv_buffer_get_status(p_arv_buffer) == ARV_BUFFER_STATUS_SUCCESS;
    bool buffer_pool    = (bool)stream.p_buffer_pool;
    if (!buffer_success || !buffer_pool)
    {
        if (!buffer_success)
        {
            RCLCPP_WARN(p_ca_instance->logger_,
                        "(%s) Frame error: %s",
                        stream.sensor.frame_id.c_str(),
                        arvBufferStatusFromInt[arv_buffer_get_status(p_arv_buffer)]);
        }

        arv_stream_push_buffer(p_stream, p_arv_buffer);
        return;
    }

    //--- push buffer pointers into concurrent queue to be processed by processing thread
    auto p_img_msg = (*stream.p_buffer_pool)[p_arv_buffer];
    stream.buffer_queue.push(std::make_tuple(p_arv_buffer, p_img_msg));
}

} // end namespace camera_aravis

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis::CameraAravis)
