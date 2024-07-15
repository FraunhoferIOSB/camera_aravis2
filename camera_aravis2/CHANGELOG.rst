^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_aravis2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Implemented GenICam compliant 'camera_driver_gv' for ROS2
	* Allowing to access a GenICam compliant via its serial number and publishing an image stream
	* Supporting multiple substreams
	* Allowing to flexibly control GenICam-Specific parameters, such as TransportLayerControl, ImageFormatControl, AcquisitionControl, AnalogControl
	* Support for PTP timestamps.
	* Support to publish diagnostic messages for camera.
* Implemented 'camera_finder' to list all available GenICam devices.
* Implemented 'camera_xml_exporter' to export camera-specific GenICam support into XML.
* CI/CD support
* Contributors: Boitumelo Ruf
