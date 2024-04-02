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

#ifndef CAMERA_ARAVIS2__CONFIG_STRUCTS_H_
#define CAMERA_ARAVIS2__CONFIG_STRUCTS_H_

// Std
#include <cfloat>
#include <string>

/**
 * @brief Struct representing sensor
 */
struct Sensor
{
    /// Frame ID associated with the sensor.
    std::string frame_id = "";

    /// Width of the sensor in pixel.
    int32_t width = 0;

    /// Height of the sensor in pixel.
    int32_t height = 0;

    /// Pixel format associated ith the sensor.
    std::string pixel_format = "";

    /// Number of pixel associated with the pixel format.
    size_t n_bits_pixel = 0;

    /// Flip the image horizontally on the device.
    bool reverse_x = false;

    /// Flip the image vertically on the device.
    bool reverse_y = false;
};

/**
 * @brief Struct representing region of interest corresponding to image.
 */
struct ImageRoi
{
    /// Offset in x direction.
    int x = 0;

    /// Offset in y direction.
    int y = 0;

    /// Width of image.
    int width = 0;

    /// Minimum width of image.
    int width_min = 0;

    /// Maximum width of image.
    int width_max = INT32_MAX;

    /// Height of image.
    int height = 0;

    /// Minimum height of image.
    int height_min = 0;

    /// Maximum height of image.
    int height_max = INT32_MAX;
};

/**
 * @brief Struct holding control settings for image acquisition.
 *
 */
struct AcquisitionControl
{
    /// String representing acquisition mode of the camera.
    std::string acquisition_mode = "";

    /// Number of frames to acquire when 'MultiFrame' is selected as acquisition mode.
    int frame_count = 0;

    /// Exposure mode used for acquisition.
    std::string exposure_mode = "";

    /// Auto mode for exposure if not exposure not controlled by trigger.
    std::string exposure_auto = "";

    /// Exposure time in us when exposure mode is set to 'Timed' and auto exposure is 'Off'.
    double exposure_time = 0;

    /// Flag indicating if the setting of acquisition frame rate is enabled.
    bool is_frame_rate_enable = true;

    /// Acquisition frame rat.
    double frame_rate = 0.0;

    /// Minimum frame rate possible.
    double frame_rate_min = 0.0;

    /// Maximum frame rate possible.
    double frame_rate_max = DBL_MAX;
};

#endif  // CAMERA_ARAVIS2__CONFIG_STRUCTS_H_
