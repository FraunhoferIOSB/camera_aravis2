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

// ROS
#include <rclcpp/rclcpp.hpp>

// camera_aravis2
#include "../include/camera_aravis/genicam_xml_exporter.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_aravis2::GenICamXmlExporter>();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
