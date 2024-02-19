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

#ifndef CAMERA_ARAVIS_UTILS
#define CAMERA_ARAVIS_UTILS

// Std
#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace camera_aravis
{
// trim from start (in place)
static inline void ltrim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch)
                                    { return !std::isspace(ch); }));
}

// trim from end (in place)
static inline void rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch)
                         { return !std::isspace(ch); })
              .base(),
            s.end());
}

// trim from both ends (in place)
static inline void trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}

void parse_string_args(std::string in_arg_string, std::vector<std::string>& out_args)
{
    size_t array_start = 0;
    size_t array_end   = in_arg_string.length();
    if (array_start != std::string::npos && array_end != std::string::npos)
    {
        // parse the string into an array of parameters
        std::stringstream ss(in_arg_string.substr(array_start, array_end - array_start));
        while (ss.good())
        {
            std::string temp;
            getline(ss, temp, ',');
            trim(temp);
            out_args.push_back(temp);
        }
    }
    else
    {
        // add just the one argument onto the vector
        out_args.push_back(in_arg_string);
    }
}

} // end namespace camera_aravis

#endif // CAMERA_ARAVIS_UTILS