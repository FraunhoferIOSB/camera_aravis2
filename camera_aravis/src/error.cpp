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

#include "../include/camera_aravis/error.h"

namespace camera_aravis2
{

//==================================================================================================
GuardedGError::~GuardedGError()
{
    reset();
}

//==================================================================================================
void GuardedGError::reset()
{
    if (!err)
        return;
    g_error_free(err);
    err = nullptr;
}

//==================================================================================================
GError** GuardedGError::ref()
{
    this->reset();

    return &err;
}

//==================================================================================================
GError* GuardedGError::operator->() noexcept
{
    return err;
}

//==================================================================================================
GuardedGError::operator bool() const
{
    return nullptr != err;
}

//==================================================================================================
void GuardedGError::log(const rclcpp::Logger& logger) const
{
    if (err == nullptr)
        return;

    RCLCPP_ERROR(logger, "[%s] Code %i: %s",
                 g_quark_to_string(err->domain), err->code, err->message);
}

}  // namespace camera_aravis2
