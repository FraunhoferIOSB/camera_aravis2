# camera_aravis2

An actively maintained ROS 2 camera driver for [GenICam](https://www.emva.org/standards-technology/genicam/)-based (GigEVision and USB3Vision) cameras. 
It is a subsequent development of [camera_aravis](https://github.com/FraunhoferIOSB/camera_aravis), but in order to clean up some legacy code and, in turn, support new features mor easily, we opted to implement it with a new code-base. 
It is open sourced under the 3-clause BSD license.

It relies on the [Aravis](http://live.gnome.org/Aravis) library to access the GigEVision and USB3Vision cameras. 
Aravis is a glib/gobject based library for video acquisition using GenICam cameras. 
It currently implements the gigabit ethernet and USB3 protocols used by industrial cameras.

------------------------

### Continuous Integration:

| Service    | devel  | main |
| ---------- | ------- | ------ |
| GitHub     | [![Humble Hawksbill](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_humble.yml/badge.svg?branch=devel)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_humble.yml)<br>[![Iron Irwini](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_iron.yml/badge.svg?branch=devel)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_iron.yml) | [![Humble Hawksbill](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_humble.yml/badge.svg?branch=main)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_humble.yml)<br>[![Iron Irwini](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_iron.yml/badge.svg?branch=main)](https://github.com/FraunhoferIOSB/camera_aravis2/actions/workflows/build_and_test_iron.yml) |

------------------------

### Contents:

- [Running the Camera Driver](#running-the-camera-driver)
    - [Configuration](#configuration)
    - [Example Launch Files](#example-launch-files)
- [Finding Available Cameras](#finding-available-cameras)
- [Extracting Camera-Specific GenICam XML](#extracting-camera-specific-genicam-xml)
- [Building from Source](#building-from-source)
    - [Requirements](#requirements)
    - [Build](#build)
    - [Test](#test)
- [Known Issues](#known-issues)

------------------------

## Running the Camera Driver

In camera_aravis2 the actual camera driver is implemented in two nodes:
- `camera_driver_gv` for GigEVision cameras, and
- `camera_driver_uv`, for USB3Vision cameras. **(Currently only implemented as stub. Looking for help: [Support USB3 cameras](https://github.com/FraunhoferIOSB/camera_aravis2/issues/14))**

### Configuration

The configuration of the camera driver is divided into a driver-specific and GenICam-specific parameterization as described below.

#### Driver-specific Parameters

- `guid`: Serial number of the camera that is to be opened. GUIDs of available cameras can be discovered with [`camera_finder`](#finding-available-cameras) node.
    - Type: String
    - Default: ""
    - Optional. If omitted a random camera is picked from the list of connected cameras and will be opened.
- `stream_names`: String list of names that are to be associated with each stream. If multiple streams are available, these names will be appended to the topic names in order to distinguish the different image streams.
    - Type: String-Array
    - Default: {}
    - Optional. If omitted or less names are given than streams available, each stream will get given a name based on its ID, starting with 0.
- `camera_info_urls`: String list of urls to camera_info files associated with each stream. List should have the same length as the number of streams provided by the camera. If the number of URLs does not correspond to number of streams available, the minimum of both is used to set the number of streams that are to be established.
    - Type: String-Array
    - Default: {}
    - Optional. If omitted, it is constructed from the camera GUID located within the current working directory, with the stream name separated by '_' appended to the file name, if more than one streams are instantiated.
- `frame_id`: Frame ID that is to be associated with the sensor and, in turn,  with the image data. If multiple streams are supported by the camera, the given ID serves as a base string to which the stream name is appended, together with '_' as separator. 
    - Type: String
    - Default: ""
    - Optional. If no frame ID is specified, the name of the node will be used.

#### GenICam-Specific Parameters

To configure the camera, camera_aravis2 relies on the GenICam Standard Feature Naming Convention (SNFC) which can be found [here](https://www.emva.org/standards-technology/genicam/genicam-downloads/). 
The SNFC groups the individual features into numerous catagories (e.g. Image Format Control, Acquisition Control, Analog Control, ...).
Camera_aravis2 explicitly looks for a number of features in a couple of categories to be specified and tries to set the features accordingly. 
If specified as launch parameter, camera_aravis2 will set the following features in the same order as listed below:

- `ImageFormatControl`
    - `PixelFormat` (String): Format of the pixels provided by the device
    - `ReverseX` (Bool): Flip horizontally the image sent by the device.
    - `ReverseY` (Bool): Flip vertically the image sent by the device.
    - `BinningHorizontal` (Int): Number of pixels to horizontally combine together.
    - `BinningHorizontalMode` (String): Mode to use when BinningHorizontal is used. Values possible: 'Sum', or 'Average'.
    - `BinningVertical` (Int): Number of pixels to horizontally combine together.
    - `BinningVerticalMode` (String): Mode to use when BinningHorizontal is used. Values possible: 'Sum', or 'Average'.
    - `Width` (Int): Width of the image provided by the device (in pixels).
    - `Height` (Int): Height of the image provided by the device (in pixels).
    - `OffsetX` (Int): Horizontal offset from the origin to the region of interest (in pixels).
    - `OffsetY` (Int): Vertical offset from the origin to the region of interest (in pixels).
- `AcquisitionControl`
    - `AcquisitionMode` (String): Sets the acquisition mode of the device. Values possible: 'SingleFrame', 'MultiFrame', or 'Continuous'.
    - `AcquisitionFrameCount` (Int): Number of frames to acquire in MultiFrame Acquisition mode. Only evaluated if 'AcquisitionMode' is 'MultiFrame'.
    - `ExposureMode` (String): Sets the operation mode of the Exposure. Values possible: 'Off', 'Timed', 'TriggerWidth', or 'TriggerControlled'.
    - `ExposureAuto` (String): Sets the automatic exposure mode when ExposureMode is Timed. Values possible: 'Off', 'Once', or 'Continuous'.
    - `ExposureTime` (Double): Sets the Exposure time when 'ExposureMode' is 'Timed' and 'ExposureAuto' is 'Off'.
    - `AcquisitionFrameRateEnable` (Bool): Controls if the AcquisitionFrameRate feature is writable and used to control the acquisition rate.
    - `AcquisitionFrameRate` (Double): Controls the acquisition rate (in Hertz) at which the frames are captured.

NOTE: The example values that are given in case of string parameters are given in accordance with the GenICam SNFC.
The possible values might differ according to the actual implementation by the camera manufacturer.

When launching the camera driver, the features are to be configured as nested launch parameters.
While the parameters are evaluated by camera_aravis2 in the specific order which is listed above, they can be specified in an arbitrary order in the launch file.
<details>
<summary>For example ...</summary>

```Python
    ...
    parameters=[{
                    # Driver-specific parameters
                    ...
                    
                    # GenICam-specific parameters
                    "ImageFormatControl": {
                        "PixelFormat": "BayerRG8",
                        "Width": 1920,
                        "Height": 1200,
                    },
                    "AcquisitionControl": {
                        "ExposureMode": "Timed",
                        "ExposureAuto": "Continuous",
                        "AcquisitionFrameRate": 30.0
                    }
                }]
    ...
```
</details>
<br>

If a feature is omitted but supported by the camera, it will default to the value that is currently set on the device.

To find the features that are available for configuration on the camera, together with their feature names and their value ranges, look into the documentation of the manufacturer or [extract the camera-specific GenICam XML file](#extracting-camera-specific-genicam-xml).

### Example Launch Files

- ```camera_driver_gv```: [camera_aravis2/launch/camera_driver_gv_example.launch.py](./camera_aravis2/launch/camera_driver_gv_example.launch.py)

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

## Extracting Camera-specific GenICam XML

Each camera model has a specific XML data stored inside the device memory which describes the GenICam interface of the camera. 
In this XML data the supported features of the camera are documented and can help to configure the camera.
To extract this XML data and write it into a file, camera_aravis2 provides the node `camera_xml_exporter` which can be invoked as shown below:

```bash
ros2 run camera_aravis2 camera_xml_exporter --ros-args -p guid:=<camera_guid> -p xml_file:=<output_file>
```

If `guid` is omitted, the XML data will be read from any of the cameras which are connected and found by camera_aravis2.
As a `xml_file`, either a relative or absolute path can be provided in which the XML data is to be saved.
If omitted, the data is saved into a file in the current working directory with the 'guid' as filename.

Example launch file: [camera_aravis2/launch/camera_xml_exporter_example.launch.py](./camera_aravis2/launch/camera_xml_exporter_example.launch.py).


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
- ROS 2 (tested with [humble and later](#continuous-integration))
- cv_bridge
- image_transport
- camera_info_manager
- std_msgs
- sensor_msgs
- camera_aravis2_msgs

**camera_aravis2_msgs**:

- ROS 2 (tested with [humble and later](#continuous-integration))
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
colcon build --symlink-install --packages-up-to camera_aravis2
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