# camera_aravis2

An actively maintained ROS2 camera driver for [GenICam](https://www.emva.org/standards-technology/genicam/)-based (GigEVision and USB3Vision) cameras. 
It is a subsequent development of [camera_aravis](https://github.com/FraunhoferIOSB/camera_aravis), but in order to clean up some legacy code and, in turn, support new features mor easily, we opted to implement it with a new code-base. 
It is open source under the 3-clause BSD license.

It relies on the [Aravis](http://live.gnome.org/Aravis) library to access the GigEVision and USB3Vision cameras. 
Aravis is a glib/gobject based library for video acquisition using GenICam cameras. 
It currently implements the gigabit ethernet and USB3 protocols used by industrial cameras.
It is licensed under LGPLv2+.

------------------------

### Continuous Integration:

| Service    | devel  | main |
| ---------- | ------- | ------ |
| GitHub     | [![Iron Irwini](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test.yml/badge.svg?branch=devel)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test.yml)    | [![Iron Irwini](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test.yml/badge.svg?branch=main)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test.yml) |

------------------------

### Contents:

- [Running the Camera Driver](#running-the-camera-driver)
- [Finding Available Cameras](#finding-available-cameras)
- [Extracting Camera-Specific GenICam XML](#extracting-camera-specific-genicam-xml)
- [Building from Source](#building-from-source)
    - [Requirements](#requirements)
    - [Build](#build)
    - [Test](#test)
- [Known Issues](#known-issues)
------------------------

## Running the Camera Driver

## Finding Available Cameras

Camera_aravis2 provides the node `camera_finder` to find available cameras and print out the corresponding GUIDs.
These, in turn, are needed to run the camera drivers or to export the camera specific GenICam XML.

```bash
ros2 run camera_aravis2 camera_finder
```

Alternatively, you can use `aravis-tools`: 

```
sudo apt install aravis-tools
arv-tool-0.8
```

## Extracting Camera-Specific GenICam XML

Each camera model has a specific XML data stored inside the device memory which describes the GenICam interface of the camera. 
In this XML data the supported features of the camera are documented and can help to configure the camera.
To extract this XML data and write it into a file, camera_aravis2 provides the node `camera_xml_exporter` which can be invoked as shown below:

```bash
ros2 run camera_aravis2 camera_xml_exporter --ros-args -p guid:=<camera_guid> -p xml_file:=<output_file>
```

If `guid` is omitted, the XML data will be read from any of the cameras which are connected and found by camera_aravis.
As a `xml_file`, either a relative or absolute path can be provided in which the XML data is to be saved.
If omitted, the data is saved into a file in the current working directory with the 'guid' as filename.


Alternatively, you can use `aravis-tools` to see the feature list and the XML file: 

```
sudo apt install aravis-tools
arv-tool-0.8 --name=<camera_guid> features
arv-tool-0.8 --name=<camera_guid> genicam
```

## Building from Source

### Requirements

**camera_aravis2**:

- Aravis 0.8 or later
- ROS2
- cv_bridge
- image_transport
- camera_info_manager
- std_msgs
- sensor_msgs
- camera_aravis2_msgs

**camera_aravis2_msgs**:

- ROS2
- std_msgs
- sensor_msgs

### Build

Initialize `rosdep` and install dependencies:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Run `colcon` to build from source:

```bash
colcon build --symlink-install
```

To build in 'Debug' mode add `--cmake-args -DCMAKE_BUILD_TYPE=Debug` to colcon command.
If omitted, camera_aravis2 will be build in 'Release' mode.

### Test

To run tests after build, additionally call:

```bash
colcon test --event-handlers=console_direct+
colcon test-result --all

```

## Known Issues

- **Permanent acquisition and publication of image data:**
With camera_aravis for ROS1, the acquisition and publication of the image data only started when a subscriber has been registered on the image topic and stopped as soon as no subscriber has been registered anymore.
This helped to minimize unnecessary data transfer. 
With the so-called [`matched-events`](https://docs.ros.org/en/foxy/Releases/Release-Iron-Irwini.html#matched-events), ROS Iron Irwini has introduced a functionality that allows to do the same. 
However, since the `image_transport` package for ROS Iron, which is used in `camera_aravis`, does not yet implement this feature.
This is implemented in `rolling`, however, and is intended for the next release.