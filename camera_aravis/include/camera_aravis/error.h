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

#ifndef CAMERA_ARAVIS__ERROR_H_
#define CAMERA_ARAVIS__ERROR_H_

// GLib
#include <glib-2.0/glib.h>

// Std
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>

/// Macro to assert success and log GError if necessary
#define ASSERT_GERROR(err, logger, success) \
    if (err)                                \
    {                                       \
        success &= false;                   \
        err.log(logger);                    \
    }                                       \
    else                                    \
    {                                       \
        success &= true;                    \
    }

/// Macro to check if error occurred and log if necessary
#define CHECK_GERROR(err, logger) \
    if (err)                      \
        err.log(logger);

namespace camera_aravis2
{

class GuardedGError
{
    //--- METHOD DECLARATION ---//
  public:
    GuardedGError() = default;

    ~GuardedGError();

    void reset();

    GError** ref();

    GError* operator->() noexcept;

    operator bool() const;

    void log(const rclcpp::Logger& logger) const;

    //--- MEMBER DECLARATION ---//

  private:
    GError* err = nullptr;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS__ERROR_H_
