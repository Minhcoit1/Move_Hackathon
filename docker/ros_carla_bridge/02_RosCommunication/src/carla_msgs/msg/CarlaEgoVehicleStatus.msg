#
# Copyright (c) 2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

std_msgs/Header header

float32 velocity
geometry_msgs/Accel acceleration
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 location
CarlaEgoVehicleRotation rotation

# the current control values, as reported by Carla
CarlaEgoVehicleControl control
