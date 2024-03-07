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

#ifndef CAMERA_ARAVIS__CAMERA_ARAVIS_H_
#define CAMERA_ARAVIS__CAMERA_ARAVIS_H_

// Std
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// Aravis
extern "C"
{
#include "aravis-0.8/arv.h"
}

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

// camera_aravis
#include "camera_aravis/camera_buffer_pool.h"
#include "camera_aravis/concurrent_queue.hpp"
#include "camera_aravis/conversion_utils.h"
#include "camera_aravis/error.hpp"

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
    //--- STRUCT DECLARATION ---//

    /**
     * @brief Struct implementing Sensor
     */
    struct Sensor
    {
        Sensor() :
          frame_id(""),
          width(0),
          height(0),
          pixel_format(""),
          n_bits_pixel(0)
        {
        }

        /// Frame ID associated with the sensor.
        std::string frame_id;

        /// Width of the sensor in pixel.
        int32_t width;

        /// Height of the sensor in pixel.
        int32_t height;

        /// Pixel format associated ith the sensor.
        std::string pixel_format;

        /// Number of pixel associated with the pixel format.
        size_t n_bits_pixel = 0;
    };

    /**
     * @brief Struct implementing camera stream.
     */
    struct Stream
    {
        /**
         * @brief Default constructor
         */
        Stream() :
          p_arv_stream(nullptr),
          p_buffer_pool(CameraBufferPool::SharedPtr()),
          name(""),
          sensor(Sensor()),
          p_camera_info_manager(nullptr),
          p_cam_info_msg(nullptr),
          is_buffer_processed(false)
        {
        }

        /// Pointer to aravis stream.
        ArvStream* p_arv_stream;

        /// Shared pointer to buffer pool.
        CameraBufferPool::SharedPtr p_buffer_pool;

        /// Name of stream.
        std::string name;

        /// Sensor associated with the stream.
        Sensor sensor;

        /// URL to camera info yaml file.
        std::string camera_info_url;

        /// Conversion function to convert pixel format from sensor into image message.
        ConversionFunction cvt_pixel_format;

        /// Camera publisher.
        image_transport::CameraPublisher camera_pub;

        /// Unique pointer to camera info manager.
        std::unique_ptr<camera_info_manager::CameraInfoManager> p_camera_info_manager;

        /// Pointer to camera_info message.
        sensor_msgs::msg::CameraInfo::SharedPtr p_cam_info_msg;

        /// Flag controlling processing of buffer data.
        bool is_buffer_processed;

        /// Thread to process ready stream buffer.
        std::thread buffer_processing_thread;

        /// Concurrent queue holding the buffer data to be processed in a separate thread.
        ConcurrentQueue<std::tuple<ArvBuffer*, sensor_msgs::msg::Image::SharedPtr>>
          buffer_queue;
    };

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization constructor
     *
     * @param[in] options Node options.
     */
    explicit CameraAravis(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     *
     */
    ~CameraAravis() override;

    /**
     * @brief Returns true, if node is spawning or is initialized. False, otherwise.
     */
    bool is_spawning_or_initialized() const;

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
     * @brief Set up camera stream structs. Here, the number of streams available are discovered,
     * and user settings are read from launch parameters and are used to set up structs. This also
     * involves the logic of how to handle faults of inconsistent number of stream_names,
     * pixel_formats, and camera_info settings.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool set_up_camera_stream_structs();

    /**
     * @brief Initialize settings revolving around the pixel formats, i.e. conversion functions and
     * bits per pixel, and set parameters on camera accordingly.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool initialize_and_set_pixel_formats();

    /**
     * @brief Set acquisition control settings.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool set_acquisition_control_settings();

    /**
     * @brief Discover number of available camera streams.
     */
    int discover_stream_number();

    /**
     * @brief Spawn camera streams.
     */
    void spawn_camera_streams();

    /**
     * @brief Tune specific parameters for GigEVision streams.
     *
     * @param[in] p_stream Pointer to Aravis Gv stream.
     */
    void tune_gv_stream(ArvGvStream* p_stream) const;

    /**
     * @brief Process available stream buffer.
     *
     * @param[in] stream_id ID of stream for which the buffer is to be processed.
     */
    void process_stream_buffer(const uint stream_id);

    /**
     * @brief Set metadata to image message.
     *
     * @param[in,out] p_img_msg Pointer to image message.
     * @param[in] p_buffer Pointer to aravis buffer holding the pixel data.
     * @param[in] sensor Sensor object corresponding to image. Used to set frame_id, image_encoding,
     * and more.
     */
    void fill_image_msg_metadata(sensor_msgs::msg::Image::SharedPtr& p_img_msg,
                                 ArvBuffer* p_buffer,
                                 const Sensor& sensor) const;

    /**
     * @brief Fill camera_info message.
     *
     * @param[in,out] stream Stream object which holds camera_info message and other data.
     * @param[in] p_img_msg Pointer to corresponding image message.
     */
    void fill_camera_info_msg(Stream& stream,
                              const sensor_msgs::msg::Image::SharedPtr& p_img_msg) const;

    /**
     * @brief Print stream statistics, such as completed and failed buffers.
     */
    void print_stream_statistics() const;

    //--- FUNCTION DECLARATION ---//

  protected:
    /**
     * @brief Construct GUID string of given camera, using vendor name, model name and
     * device serial number.
     *
     * @return GUID in the format: <vendor_name>-<model_name>-<device_sn | device_id>.
     */
    static inline std::string construct_camera_guid_str(ArvCamera* p_cam);

    /**
     * @brief Handle 'control-lost' signal emitted by aravis.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handle_control_lost_signal(ArvDevice* p_device, gpointer p_user_data);

    /**
     * @brief Handle 'new-buffer' signal emitted by aravis, notifying that a new buffer is ready.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handle_new_buffer_signal(ArvStream* p_stream, gpointer p_user_data);

    //--- MEMBER DECLARATION ---//
  private:
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

    /// List of camera streams
    std::vector<Stream> streams_;

    /// Atomic flag, indicating if streams are being spawned.
    std::atomic<bool> is_spawning_;

    /// Thread in which the streams are spawned.
    std::thread spawn_stream_thread_;

    /// List of pointers to data tuples for the new-buffer callback.
    std::vector<std::shared_ptr<std::tuple<CameraAravis*, uint>>> new_buffer_cb_data_ptrs;

    /// Flag indicating to use PTP timestamp.
    bool use_ptp_timestamp_;
};

}  // namespace camera_aravis

#endif  // CAMERA_ARAVIS__CAMERA_ARAVIS_H_
