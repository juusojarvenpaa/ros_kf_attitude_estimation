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
Description: This script is used to calibrate the MPU6050 sensor.
The script reads the raw accelerometer and gyroscope data from the sensor
and prints it to the console. The data is used to calculate the offset
values for the sensor. The offset values are used to calibrate the sensor.
"""

import smbus
from math import radians
import numpy
from time import sleep

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

    return GxSI, GySI, GzSI, Gx, Gy, Gz, AxSI, AySI, AzSI, Ax, Ay, Az

if __name__ == '__main__':
    bus = smbus.SMBus(1)
    Device_Address = 0x68

    MPU_Init()

    Gx_values = []
    Gy_values = []
    Gz_values = []
    GxSI_values = []
    GySI_values = []
    GzSI_values = []
    Ax_values = []
    Ay_values = []
    Az_values = []
    AxSI_values = []
    AySI_values = []
    AzSI_values = []

    for i in range(1000):
        GxSI, GySI, GzSI, Gx, Gy, Gz, AxSI, AySI, AzSI, Ax, Ay, Az = imu_data()
        print("Gx: %.2f, Gy: %.2f, Gz: %.2f" % (GxSI, GySI, GzSI))
        print("Ax: %.2f, Ay: %.2f, Az: %.2f" % (Ax, Ay, Az))

        Gx_values.append(Gx)
        Gy_values.append(Gy)
        Gz_values.append(Gz)
        GxSI_values.append(GxSI)
        GySI_values.append(GySI)
        GzSI_values.append(GzSI)
        Ax_values.append(Ax)
        Ay_values.append(Ay)
        Az_values.append(Az)
        AxSI_values.append(AxSI)
        AySI_values.append(AySI)
        AzSI_values.append(AzSI)

        sleep(0.1)

    Gx_offset = numpy.mean(Gx_values)
    Gy_offset = numpy.mean(Gy_values)
    Gz_offset = numpy.mean(Gz_values)
    GxSI_offset = numpy.mean(GxSI_values)
    GySI_offset = numpy.mean(GySI_values)
    GzSI_offset = numpy.mean(GzSI_values)
    Ax_offset = numpy.mean(Ax_values)
    Ay_offset = numpy.mean(Ay_values)
    Az_offset = numpy.mean(Az_values)
    AxSI_offset = numpy.mean(AxSI_values)
    AySI_offset = numpy.mean(AySI_values)
    AzSI_offset = numpy.mean(AzSI_values)

    # Print angular velocity offsets in degrees/s and rad/s
    print("Gx offset: %.4f deg/s, %.4f rad/s" % (Gx_offset, GxSI_offset))
    print("Gy offset: %.4f deg/s, %.4f rad/s" % (Gy_offset, GySI_offset))
    print("Gz offset: %.4f deg/s, %.4f rad/s" % (Gz_offset, GzSI_offset))

    # Print linear acceleration offsets in m/s^2 and g/s
    print("Ax offset: %.4f m/s^2, %.4f g" % (AxSI_offset, Ax_offset))
    print("Ay offset: %.4f m/s^2, %.4f g" % (AySI_offset, Ay_offset))
    print("Az offset: %.4f m/s^2, %.4f g" % (AzSI_offset - 9.80655, Az_offset - 1))
