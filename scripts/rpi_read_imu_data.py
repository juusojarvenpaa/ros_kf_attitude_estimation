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
#
# Some portions of the code are adapted from:
# "Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python"
# from [Electronic Wings](http://www.electronicwings.com).


"""
This is the script running on the Raspberry Pi that reads the IMU data from
the MPU6050 sensor and publishes it to the topic 'imu_data'.
"""

import smbus
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from math import radians, asin, cos, sin, atan2
import numpy

# Register addresses
PWR_MGMT1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Constants
G = 9.80665

# Initialize the MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# Read raw 16-bit value
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value - 65536
    return value

def imu_data():
    # Raw Accelerometer and Gyroscope data
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Accelerometer data in g
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0

    # Accelerometer data in m/s^2
    AxSI = Ax*9.80665
    AySI = Ay*9.80665
    AzSI = Az*9.80665

    # Gyroscope data in degrees/s
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0

    # Gyroscope data in rad/s
    GxSI = radians(Gx)
    GySI = radians(Gy)
    GzSI = radians(Gz)

    return GxSI, GySI, GzSI, AxSI, AySI, AzSI

# Convert Euler angles to quaternion
def ea2q(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0, 0, 0, 0]
    q[0] = cr * cp * cy + sr * sp * sy
    q[1] = sr * cp * cy - cr * sp * sy
    q[2] = cr * sp * cy + sr * cp * sy
    q[3] = cr * cp * sy - sr * sp * cy

    return q

# Convert quaternion to Euler angles in sequence (1,2,3)
def q2ea123(q):
    roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
    pitch = -asin(2*(q[1]*q[3] - q[0]*q[2]))
    yaw = atan2(2*(q[1]*q[2] + q[0]*q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)

    return roll, pitch, yaw

def ea_from_acc(AxSI, AySI, AzSI):
    # Rounding the accelerometer data to avoid numerical errors
    acc_x = round(AxSI/G, 4)
    acc_y = round(AySI/G, 4)
    acc_z = round(AzSI/G, 4)
    # Pitch is rotation around y-axis
    # Asin requires a value between -1 and 1 to avoid numerical errors
    if acc_x > 1:
        acc_x = 1
    elif acc_x < -1:
        acc_x = -1
    pitch = asin(acc_x)
    # Roll is rotation around x-axis
    roll = atan2(-acc_y, acc_z)

    return roll, pitch

def init_kalman():
    # Initial state (quaternion)
    x = numpy.array([1, 0, 0, 0])
    # Initial state covariance matrix
    P = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # Process noise covariance matrix
    Q = numpy.array([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]])
    # Measurement noise covariance matrix
    R = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # State-to-measurement matrix
    H = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    return x, P, Q, R, H

def get_measurements_with_acc_data(yaw, AxSI, AySI, AzSI):
    # Euler angles from accelerometer data
    roll, pitch = ea_from_acc(AxSI, AySI, AzSI)
    # Convert Euler angles to quaternion
    q = ea2q(roll, pitch, yaw)

    z = numpy.array([q[0], q[1], q[2], q[3]])

    return z, pitch

def kalman_filter(x, P, Q, R, H, K, GxSI, GySI, GzSI, yaw, AxSI, AySI, AzSI):
    # Corrected gyroscope and accelerometer data with measured offsets
    GxSI = GxSI + 0.0025
    GySI = GySI - 0.0003
    GzSI = GzSI - 0.0011
    AxSI = AxSI - 0.2871
    AySI = AySI - 0.0498

    # Time step
    dt = 0.1

    B = numpy.array([[0, -GxSI, -GySI, -GzSI], [GxSI, 0, GzSI, -GySI], [GySI, -GzSI, 0, GxSI], [GzSI, GySI, -GxSI, 0]])
    # State transition matrix
    A = numpy.eye(4) + 0.5*dt*B
    # Measurement matrix
    # Accelerometer data is not to be used if Az is near 0
    z, pitch = get_measurements_with_acc_data(yaw, AxSI, AySI, AzSI)

    # Prediction step
    xp = numpy.dot(A, x)
    Pp = numpy.dot(numpy.dot(A, P), numpy.transpose(A)) + Q

    # Correction step
    if pitch < 85 and pitch > -85:
        K = numpy.dot(numpy.dot(Pp, numpy.transpose(H)), numpy.linalg.inv(numpy.dot(numpy.dot(H, Pp), numpy.transpose(H)) + R))
        x = xp + numpy.dot(K, (z - numpy.dot(H, xp)))
    else:
        x = xp
    P = Pp - numpy.dot(numpy.dot(K, H), Pp)

    return x, P, K


def talker():
    pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    x, P, Q, R, H = init_kalman()
    K = 0
    yaw = 0

    # Reads IMU data from the IMU sensor and publishes it to the topic 'imu_data'
    while not rospy.is_shutdown():
        GxSI, GySI, GzSI, AxSI, AySI, AzSI = imu_data()

        # Kalman filter
        x, P, K = kalman_filter(x, P, Q, R, H, K, GxSI, GySI, GzSI, yaw, AxSI, AySI, AzSI)

        # Convert quaternion to Euler angles
        roll, pitch, yaw = q2ea123(x)

        # Publish the IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"
        imu_msg.angular_velocity = Vector3(GxSI, GySI, GzSI)
        imu_msg.linear_acceleration = Vector3(AxSI, AySI, AzSI)
        imu_msg.orientation = Quaternion(x[1], x[2], x[3], x[0])
        pub.publish(imu_msg)

        rate.sleep()

if __name__ == '__main__':
    bus = smbus.SMBus(1)
    Device_Address = 0x68

    MPU_Init()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass