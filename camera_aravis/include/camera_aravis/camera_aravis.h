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

#ifndef CAMERA_ARAVIS_CAMERA_ARAVIS
#define CAMERA_ARAVIS_CAMERA_ARAVIS

// Std
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

// TBB
// TODO: Swap TBB Concurrent Queue with own implementation
#include <tbb/concurrent_queue.h>

// Aravis
extern "C"
{
#include "arv.h"
}

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

// camera_aravis
#include "camera_buffer_pool.h"
#include "error.hpp"

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
    //--- STRUCT DECLARATION ---//

    /**
     * @brief Struct implementing camera stream.
     * Consisting of pointer to aravis stream and a associated buffer pool.
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
          frame_id(""),
          p_camera_info_manager(nullptr),
          is_buffer_processed(false)
        {
        }

        /**
         * @brief Initialization constructor
         *
         * @param[in] iName Stream name.
         */
        Stream(const std::string& iName) :
          Stream()
        {
            name = iName;
        }

        /// Pointer to aravis stream.
        ArvStream* p_arv_stream;

        /// Shared pointer to buffer pool.
        CameraBufferPool::SharedPtr p_buffer_pool;

        /// Name of stream.
        std::string name;

        /// Frame ID corresponding to the image stream
        std::string frame_id;

        /// Camera publisher.
        image_transport::CameraPublisher camera_pub;

        /// Unique pointer to camera info manager.
        std::unique_ptr<camera_info_manager::CameraInfoManager> p_camera_info_manager;

        /// Flag controlling processing of buffer data.
        bool is_buffer_processed;

        /// Thread to process ready stream buffer.
        std::thread buffer_processing_thread;

        // /// Mutex guarding concurrent access to stream buffer.
        // std::mutex buffer_mutex;

        tbb::concurrent_queue<ArvBuffer*> buffer_queue;
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
     * @brief Initialize camera stream. Get number of streams available, and initialize structs.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool initialize_camera_streams();

    /**
     * @brief Discover features available on the camera.
     */
    void discover_features();

    /**
     * @brief Spawn camera streams.
     */
    void spawn_camera_streams();

    /**
     * @brief Tune specific parameters for GigEVision streams.
     *
     * @param[in] p_stream Pointer to Aravis Gv stream.
     */
    void tuneGvStream(ArvGvStream* p_stream) const;

    /**
     * @brief Process available stream buffer.
     *
     * @param[in] stream_id ID of stream for which the buffer is to be processed.
     */
    void process_stream_buffer(const uint stream_id);

    /**
     * @brief Print stream statistics, such as completed and failed buffers.
     */
    void printStreamStatistics() const;

    //--- FUNCTION DECLARATION ---//

  protected:
    /**
     * @brief Handle 'control-lost' signal emitted by aravis.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handleControlLost(ArvDevice* p_device, gpointer p_user_data);

    /**
     * @brief Handle 'new-buffer' signal emitted by aravis, notifying that a new buffer is ready.
     *
     * @param[in] p_device Pointer to aravis device.
     * @param[in] p_user_data Pointer to associated user data.
     */
    static void handleNewBufferReady(ArvStream* p_stream, gpointer p_user_data);

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

    /// List of camera streams
    std::vector<Stream> streams_;

    /// Atomic flag, indicating if streams are being spawned.
    std::atomic<bool> is_spawning_;

    /// Thread in which the streams are spawned.
    std::thread spawn_stream_thread_;

    /// List of pointers to data tuples for the new-buffer callback
    std::vector<std::shared_ptr<std::tuple<CameraAravis*, uint>>> new_buffer_cb_data_ptrs;

    /// TODO: Deprecated?
    bool verbose_;

    /// TODO: Deprecated?
    std::unordered_map<std::string, const bool> implemented_features_;
};

} // namespace camera_aravis

#endif // CAMERA_ARAVIS_CAMERA_ARAVIS