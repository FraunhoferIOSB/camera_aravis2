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

#ifndef CAMERA_ARAVIS__CAMERA_BUFFER_POOL_H_
#define CAMERA_ARAVIS__CAMERA_BUFFER_POOL_H_

// Std
#include <map>
#include <memory>
#include <mutex>
#include <stack>

// Aravis
extern "C"
{
#include "aravis-0.8/arv.h"
}

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace camera_aravis
{

class CameraBufferPool : public std::enable_shared_from_this<CameraBufferPool>
{
  public:
    typedef std::shared_ptr<CameraBufferPool> SharedPtr;
    typedef std::weak_ptr<CameraBufferPool> WeakPtr;

    // Note: If the CameraBufferPool is destroyed, buffers will be deallocated. Therefor, make sure
    // that the CameraBufferPool stays alive longer than the given stream object.
    //
    // logger: logger object
    // stream: weakly managed pointer to the stream. Used to register all allocated buffers
    // payload_size_bytes: size of a single buffer
    // n_preallocated_buffers: number of initially allocated and registered buffers
    CameraBufferPool(const rclcpp::Logger& logger, ArvStream* stream,
                     size_t payload_size_bytes, size_t n_preallocated_buffers = 2);
    virtual ~CameraBufferPool();

    // Get an image whose lifespan is administrated by this pool (but not registered to the camera).
    sensor_msgs::msg::Image::SharedPtr getRecyclableImg();

    // Get the image message which wraps around the given ArvBuffer.
    //
    // If this buffer is not administrated by this CameraBufferPool,
    // a new image message is allocated and the contents of the buffer
    // are copied to it.
    sensor_msgs::msg::Image::SharedPtr operator[](ArvBuffer* buffer);

    inline size_t getAllocatedSize() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return n_buffers_;
    }

    inline size_t getUsedSize() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return used_buffers_.size();
    }

    inline size_t getPayloadSize() const
    {
        return payload_size_bytes_;
    }

    // Allocate new buffers which are wrapped by an image message and
    // push them to the internal aravis stream.
    void allocateBuffers(size_t n = 1);

  protected:
    // Custom deleter for aravis buffer wrapping image messages, which
    // either pushes the buffer back to the aravis stream cleans it up
    // when the CameraBufferPool is gone.
    static void reclaim(const WeakPtr& self, sensor_msgs::msg::Image* p_img);

    // Push the buffer inside the given image back to the aravis stream,
    // remember the corresponding image message.
    void push(sensor_msgs::msg::Image* p_img);

    ArvStream* stream_         = NULL;
    size_t payload_size_bytes_ = 0;
    size_t n_buffers_          = 0;

    std::map<const uint8_t*, sensor_msgs::msg::Image::SharedPtr> available_img_buffers_;
    std::map<sensor_msgs::msg::Image*, ArvBuffer*> used_buffers_;
    std::stack<sensor_msgs::msg::Image::SharedPtr> dangling_imgs_;
    mutable std::mutex mutex_;
    SharedPtr self_;

    rclcpp::Logger logger_;
};

}  // namespace camera_aravis

#endif  // CAMERA_ARAVIS__CAMERA_BUFFER_POOL_H_
