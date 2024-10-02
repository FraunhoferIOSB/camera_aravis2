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

#ifndef CAMERA_ARAVIS2__CAMERA_DRIVER_GV_H_
#define CAMERA_ARAVIS2__CAMERA_DRIVER_GV_H_

// Yaml-cpp
#include <yaml-cpp/yaml.h>

// Std
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
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

// camera_aravis2
#include "camera_aravis2/camera_aravis_node_base.h"
#include "camera_aravis2/concurrent_queue.hpp"
#include "camera_aravis2/config_structs.h"
#include "camera_aravis2/conversion_utils.h"
#include "camera_aravis2/image_buffer_pool.h"
#include <camera_aravis2_msgs/msg/camera_diagnostics.hpp>

namespace camera_aravis2
{
class CameraDriverGv : public CameraAravisNodeBase
{
    //--- STRUCT DECLARATION ---//

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
          p_buffer_pool(ImageBufferPool::SharedPtr()),
          name(""),
          sensor(Sensor()),
          image_roi(ImageRoi()),
          camera_info_url(""),
          p_camera_info_manager(nullptr),
          p_cam_info_msg(nullptr),
          is_buffer_processed(false)
        {
        }

        /// Pointer to aravis stream.
        ArvStream* p_arv_stream;

        /// Shared pointer to buffer pool.
        ImageBufferPool::SharedPtr p_buffer_pool;

        /// Name of stream.
        std::string name;

        /// Sensor associated with the stream.
        Sensor sensor;

        /// Image region associated with the stream.
        ImageRoi image_roi;

        /// Control settings for image acquisition.
        AcquisitionControl acquisition_control;

        /// Control settings for analog control.
        AnalogControl analog_control;

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
        ConcurrentQueue<std::pair<ArvBuffer*, sensor_msgs::msg::Image::SharedPtr>>
          buffer_queue;
    };

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization constructor
     *
     * @param[in] options Node options.
     */
    explicit CameraDriverGv(const rclcpp::NodeOptions& options =
                              rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     *
     */
    virtual ~CameraDriverGv();

    /**
     * @brief Returns true, if node is spawning or is initialized. False, otherwise.
     */
    bool isSpawningOrInitialized() const;

  protected:
    /**
     * @brief Set up the launch parameters.
     */
    void setUpParameters() override;

    /**
     * @brief Set up camera stream structs. Here, the number of streams available are discovered,
     * and user settings are read from launch parameters and are used to set up structs. This also
     * involves the logic of how to handle faults of inconsistent number of stream_names,
     * pixel_formats, and camera_info settings.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setUpCameraStreamStructs();

    /**
     * @brief Get list of parameters underneath 'param_name' within the list of device
     * control parameters.
     *
     * @param[in] param_name Name of the nested parameter within device control.
     * The method will prepend 'DeviceControl.' to the parameter prior to the search.
     * @param[out] param_values List of parameter values associated with feature names.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getDeviceControlParameterList(
      const std::string& param_name,
      std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const;

    /**
     * @brief Set device control settings of the camera.
     *
     * For example: DeviceLinkThroughputLimit ...
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setDeviceControlSettings();

    /**
     * @brief Get parameter with 'param_name' within the list of transport layer control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within transport layer control.
     * The method will prepend 'TransportLayerControl.' to the parameter prior to the search.
     * @param[out] param_value Parameter value.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getTransportLayerControlParameter(
      const std::string& param_name,
      rclcpp::ParameterValue& param_value) const;

    /**
     * @brief Get list of parameters underneath 'param_name' within the list of transport layer
     * control parameters.
     *
     * @param[in] param_name Name of the nested parameter within transport layer control.
     * The method will prepend 'TransportLayerControl.' to the parameter prior to the search.
     * @param[out] param_values List of parameter values associated with feature names.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getTransportLayerControlParameterList(
      const std::string& param_name,
      std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const;

    /**
     * @brief Set transport layer control settings of the camera.
     *
     * For example: PtpEnable, GevSCPSPacketSize ...
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setTransportLayerControlSettings();

    /**
     * @brief Get parameter with 'param_name' within the list of image format control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within image format control.
     * The method will prepend 'ImageFormatControl.' to the parameter prior to the search.
     * @param[out] param_value Parameter value.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getImageFormatControlParameter(
      const std::string& param_name,
      rclcpp::ParameterValue& param_value) const;

    /**
     * @brief Get list of parameters underneath 'param_name' within the list of image format
     * control parameters.
     *
     * @param[in] param_name Name of the nested parameter within image format control.
     * The method will prepend 'ImageFormatControl.' to the parameter prior to the search.
     * @param[out] param_values List of parameter values associated with feature names.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getImageFormatControlParameterList(
      const std::string& param_name,
      std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const;

    /**
     * @brief Set image format control settings of the camera.
     *
     * For example: Pixel Format, Image Size, Image Offset ...
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setImageFormatControlSettings();

    /**
     * @brief Get parameter with 'parameter_name' within the list of acquisition control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within acquisition control.
     * The method will prepend 'AcquisitionControl.' to the parameter prior to the search.
     * @param[out] param_value Parameter value.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getAcquisitionControlParameter(
      const std::string& param_name,
      rclcpp::ParameterValue& param_value) const;

    /**
     * @brief Get list of parameters underneath 'param_name' within the list of acquisition control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within acquisition control.
     * The method will prepend 'AcquisitionControl.' to the parameter prior to the search.
     * @param[out] param_values List of parameter values associated with feature names.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getAcquisitionControlParameterList(
      const std::string& param_name,
      std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const;

    /**
     * @brief Set acquisition control settings of the camera.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setAcquisitionControlSettings();

    /**
     * @brief Get parameter with 'parameter_name' within the list of analog control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within analog control.
     * The method will prepend 'AnalogControl.' to the parameter prior to the search.
     * @param[out] param_value Parameter value.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getAnalogControlParameter(
      const std::string& param_name,
      rclcpp::ParameterValue& param_value) const;

    /**
     * @brief Get list of parameters underneath 'param_name' within the list of analog control
     * parameters.
     *
     * @param[in] param_name Name of the nested parameter within analog control.
     * The method will prepend 'AnalogControl.' to the parameter prior to the search.
     * @param[out] param_values List of parameter values associated with feature names.
     * @return True if parameter is found in 'parameter_overrides_' and, thus, given by the user.
     * False otherwise.
     */
    [[nodiscard]] bool getAnalogControlParameterList(
      const std::string& param_name,
      std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const;

    /**
     * @brief Set analog control settings of the camera.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool setAnalogControlSettings();

    /**
     * @brief Discover number of available camera streams.
     */
    int discoverNumberOfStreams();

    /**
     * @brief Spawn camera streams.
     */
    void spawnCameraStreams();

    /**
     * @brief Tune specific parameters for GigEVision streams.
     *
     * @param[in] p_stream Pointer to Aravis Gv stream.
     */
    void tuneGvStream(ArvGvStream* p_stream) const;

#ifdef WITH_MATCHED_EVENTS
    /**
     * @brief Handle change in message subscription.
     *
     * In this, the image acquisition will be started or stopped, depending on the number of
     * subscribers.
     *
     * This is called each time there is a subscriber change to the image topic, even before the
     * node is fully initialized. The stored number of subscribers is also evaluated during
     * initialization in order to start acquisition, if applicable.
     *
     * @param[in] iEventInfo Info on matched event.
     */
    void handleMessageSubscriptionChange(rclcpp::MatchedInfo& iEventInfo);
#endif

    /**
     * Set up publisher for camera diagnostics.
     *
     * In this, the YAML file configuring the diagnostics which are to be published is read and
     * parsed.
     */
    void setUpCameraDiagnosticPublisher();

    /**
     * Read and publish camera diagnostics.
     *
     * This runs in a separate thread and loops at the given rate to read the stats from the camera
     * and published them on hte appropriate topic.
     */
    void publishCameraDiagnosticsLoop(double rate) const;

    /**
     * @brief Check the status of the Precision Time Protocol and reset its clock if applicable.     *
     * The clock will only be reset if the status of PTP is "Faulty", "Disabled", or
     * "Initializing".
     */
    void checkPtp();

    /**
     * @brief Process available stream buffer.
     *
     * @param[in] stream_id ID of stream for which the buffer is to be processed.
     */
    void processStreamBuffer(const uint stream_id);

    /**
     * @brief Adjust image roi to actual image size stored in the buffer.
     *
     * @param[in, out] img_roi Image ROI.
     * @param[in] p_buffer Pointer to image buffer.
     * @returns True, if roi needed adjustment. False, otherwise.
     */
    bool adjustImageRoi(ImageRoi& img_roi, ArvBuffer* p_buffer) const;

    /**
     * @brief Set metadata to image message.
     *
     * @param[in,out] p_img_msg Pointer to image message.
     * @param[in] p_buffer Pointer to aravis buffer holding the pixel data.
     * @param[in] sensor Sensor object corresponding to image. Used to set frame_id, image_encoding,
     * and more.
     * @param[in] img_roi Image ROI.
     */
    void fillImageMsgMetadata(sensor_msgs::msg::Image::SharedPtr& p_img_msg,
                              ArvBuffer* p_buffer, const Sensor& sensor,
                              const ImageRoi& img_roi) const;

    /**
     * @brief Fill camera_info message.
     *
     * @param[in,out] stream Stream object which holds camera_info message and other data.
     * @param[in] p_img_msg Pointer to corresponding image message.
     */
    void fillCameraInfoMsg(Stream& stream,
                           const sensor_msgs::msg::Image::SharedPtr& p_img_msg) const;

    /**
     * @brief Print currently applied camera configuration.
     */
    void printCameraConfiguration() const;

    /**
     * @brief Print stream statistics, such as completed and failed buffers.
     */
    void printStreamStatistics() const;

    //--- FUNCTION DECLARATION ---//

  protected:
    /**
     * @brief Handle 'new-buffer' signal emitted by aravis, notifying that a new buffer is ready.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handleNewBufferSignal(ArvStream* p_stream, gpointer p_user_data);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Transport layer control settings.
    TransportLayerControl tl_control_;

    /// List of camera streams
    std::vector<Stream> streams_;

    /// Atomic flag, indicating if streams are being spawned.
    std::atomic<bool> is_spawning_;

    /// Thread in which the streams are spawned.
    std::thread spawn_stream_thread_;

    /// Atomic flag, indicating if diagnostics are published.
    std::atomic<bool> is_diagnostics_published_;

    /// Thead in which the publishing of the camera diagnostics runs.
    std::thread diagnostic_thread_;

    /// Pointer to publisher for camera diagnostics.
    rclcpp::Publisher<camera_aravis2_msgs::msg::CameraDiagnostics>::SharedPtr p_diagnostic_pub_;

    /// Number of subscribers currently connected to the message topic.
    int current_num_subscribers_;

    /// YAML node holding diagnostic features
    YAML::Node diagnostic_features_;

    /// List of pointers to data pair for the new-buffer callback.
    std::vector<std::shared_ptr<std::pair<CameraDriverGv*, uint>>> new_buffer_cb_data_ptrs;

    /// Message strings to warn the user of inconsistencies at summary output.
    std::vector<std::string> config_warn_msgs_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__CAMERA_DRIVER_GV_H_
