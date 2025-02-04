# ROS Kalman Filter for Attitude Estimation

This is a **ROS package** designed for attitude estimation using an **MPU6050 IMU sensor** and a **Kalman filter**. The package includes  scripts for **Raspberry Pi** to read, calibrate and publish IMU data, and a script for a receiving device to listen to IMU data.

## Requirements

The system was implemented on Ubuntu 20.04, running ROS noetic on both devices.

## Installation
### **1. Clone the Repository**
```bash
cd ~/catkin_ws/src
git clone https://github.com/juusojarvenpaa/ros_kf_attitude_estimation.git
cd ~/catkin_ws
catkin_make
```

## Usage

Follow the instructions in [my BSc thesis](https://trepo.tuni.fi/handle/10024/158058) to interface the IMU with Raspberry Pi and to set up the network between the two devices.

### **1. Calibrate the IMU**
On Raspberry Pi:
```bash
rosrun ros_kf_attitude_estimation calibration.py
```
This will calculate the offsets of the sensor measurements and print them to the console. Then you must change the offsets manually in rpi_read_imu_data.py -file.

### **2. Publish IMU Data from Raspberry Pi**
On Raspberry Pi:
```bash
rosrun ros_kf_attitude_estimation rpi_read_imu_data.py
```
Reads and processes data from MPU6050 and publishes it to /imu_data topic.

### **3. Subscribe to IMU data on the other device**
On the other device:
```bash
rosrun ros_kf_attitude_estimation imu_listener.py
```
Prints the estimated angles to the console.

## License
This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

### Attribution
Some portions of the code, such as the MPU6050 sensor initialization and data reading, were adapted from:
- "Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python" from [Electronic Wings](https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi).
