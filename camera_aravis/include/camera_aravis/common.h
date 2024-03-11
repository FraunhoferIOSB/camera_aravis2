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

#ifndef CAMERA_ARAVIS__COMMON_H_
#define CAMERA_ARAVIS__COMMON_H_

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

#endif  // CAMERA_ARAVIS__COMMON_H_
