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

#ifndef CAMERA_ARAVIS__ERROR_HPP_
#define CAMERA_ARAVIS__ERROR_HPP_

// GLib
#include <glib-2.0/glib/gerror.h>

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

namespace camera_aravis
{

class GuardedGError
{
    //--- METHOD DECLARATION ---//
  public:
    ~GuardedGError()
    {
        reset();
    }

    void reset()
    {
        if (!err)
            return;
        g_error_free(err);
        err = nullptr;
    }

    GError** ref()
    {
        this->reset();

        return &err;
    }

    GError* operator->() noexcept
    {
        return err;
    }

    operator bool() const
    {
        return nullptr != err;
    }

    void log(const rclcpp::Logger& logger) const
    {
        if (err == nullptr)
            return;

        RCLCPP_ERROR(logger, "[%s] Code %i: %s",
                     g_quark_to_string(err->domain), err->code, err->message);
    }

    friend bool operator==(const GuardedGError& lhs, const GError* rhs);
    friend bool operator==(const GuardedGError& lhs, const GuardedGError& rhs);
    friend bool operator!=(const GuardedGError& lhs, std::nullptr_t);

    //--- MEMBER DECLARATION ---//

  private:
    GError* err = nullptr;
};

bool operator==(const GuardedGError& lhs, const GError* rhs)
{
    return lhs.err == rhs;
}
bool operator==(const GuardedGError& lhs, const GuardedGError& rhs)
{
    return lhs.err == rhs.err;
}
bool operator!=(const GuardedGError& lhs, std::nullptr_t)
{
    return !!lhs;
}

}  // namespace camera_aravis

#endif  // CAMERA_ARAVIS__ERROR_HPP_
