# Contains the standard sensor_msgs/CameraInfo and additional information about the image acquisition parameters.
# This includes parameters on the exposure, gain, black level, white balance and device temperature.

sensor_msgs/CameraInfo camera_info # contains a header and calibration parameters

float64 exposure_time # in microseconds
float64 gain # unit is device-specific
float64 black_level # unit is device-specific

float64 white_balance_red # unit is device-specific
float64 white_balance_green # unit is device-specific
float64 white_balance_blue # unit is device-specific

float64 temperature # in degrees Celsius
