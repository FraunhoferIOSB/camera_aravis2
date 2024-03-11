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

#ifndef CAMERA_ARAVIS__CONCURRENT_QUEUE_HPP_
#define CAMERA_ARAVIS__CONCURRENT_QUEUE_HPP_

// Std
#include <condition_variable>
#include <mutex>
#include <queue>

namespace camera_aravis2
{

/**
 * @brief Templated class implementing a thread safe queue that can be accessed (via push and pop)
 * from multiple threads. This implementation is inspired by the `concurrent_queue` within the TBB
 * library.
 */
template <typename T>
class ConcurrentQueue
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @return True, if queue is empty. False, otherwise.
     */
    [[nodiscard]] bool empty() const noexcept
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return queue_.empty();
    }

    /**
     * @return Size of queue. I.e. number of elements stored inside queue.
     */
    [[nodiscard]] size_t size() const noexcept
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return queue_.size();
    }

    /**
     * @brief Pushed a copy of `input` into the queue container. This will induce a notification
     * via the condition variable that a new member is inside the queue.
     */
    void push(const T& input) noexcept
    {
        {
            std::lock_guard<std::mutex> lck(mtx_);
            queue_.push(input);
        }
        cond_.notify_one();
    }

    /**
     * @brief Pops a copy of the first item in the container and assigns it to `output`. The popped
     * item is destroyed. If queue  is empty, this method will wait until an item becomes available.
     */
    void pop(T& output) noexcept
    {
        std::unique_lock<std::mutex> lck(mtx_);
        cond_.wait(lck, [this]
                   { return !queue_.empty(); });
        output = queue_.front();
        queue_.pop();
    }

    //--- MEMBER DECLARATION ---//

  private:
    /// Queue object
    std::queue<T> queue_;

    /// Condition variable to notify when new object is within queue
    std::condition_variable cond_;

    /// Mutex, guarding queue
    mutable std::mutex mtx_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS__CONCURRENT_QUEUE_HPP_
