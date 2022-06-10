# Status of GenICam properties affected by auto-parameters
#
# GenICam is maintained by the european machine vision association (emva):
# https://www.genicam.org
#
# Properties are defined in GenICam Standard Features Naming Convention (SFNC).
# 
# Implementation of most properties is "optional".
# If a parameter is not implemented, it should be set to NaN.
#
# Note: Vendor specific extensions are not taken into account.
# Consider disabling such proprietary auto-parameters.

std_msgs/Header header

# ExposureAuto
float64 exposure_time

# GainAuto
float64 gain

# GainAutoBalance
float64 gain_red
float64 gain_green
float64 gain_blue

# BlackLevelAuto
float64 black_level

# BlackLevelAutoBalance
float64 bl_red
float64 bl_green
float64 bl_blue

# BalanceWhiteAuto
float64 wb_red
float64 wb_green
float64 wb_blue
