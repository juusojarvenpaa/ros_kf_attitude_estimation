#!/usr/bin/env python

# Copyright 2024 Juuso Järvenpää
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is the script that runs on the laptop to listen to the IMU data
"""

import rospy
from sensor_msgs.msg import Imu
from math import atan2, asin, degrees

def q2ea123(q):
    # Convert quaternion to Euler angles in radians
    roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
    pitch = -asin(2*(q[1]*q[3] - q[0]*q[2]))
    yaw = atan2(2*(q[1]*q[2] + q[0]*q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)

    # Convert radians to degrees
    roll = degrees(roll)
    pitch = degrees(pitch)
    yaw = degrees(yaw)

    return roll, pitch, yaw


# Callback function for the subscriber to the topic 'imu_data' 
# that prints the angular velocity and linear acceleration
def callback(data):

    roll, pitch, yaw = q2ea123([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
    print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % (roll, pitch, yaw))
    print()


def listener():
    rospy.init_node('imu_listener', anonymous=True)

    rospy.Subscriber('imu_data', Imu, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()