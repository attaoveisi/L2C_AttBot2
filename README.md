# L2C_AttBot2
This repo shows how to use Kalman filter and odometry to create data before the fusion algorithm for localization purposes.

![AttBot 2.0](https://user-images.githubusercontent.com/17289954/98923515-0924b780-24d4-11eb-8d62-ef4f5914992b.png)

## IMU message for rosserial

AttBot2.0 has BNo055 Bosch 9D sensor for measuring the 3D accelerations, 3D angular rates, and a 3D compass. It can deliver the absolute yaw, pitch, and yaw angles which are used to create the `sensor_msgs/Imu` message. The created message in Arduino DUE is communicated with the high level Raspberry Pi 4 per rosserial library.
The relative movement between the chassis of the AttBot2.0 and the cabin is assummed to be negligible. 

![image](https://user-images.githubusercontent.com/17289954/103778665-dc53d300-5032-11eb-9f9a-2b8df34e4b68.png)

The calibration of the BNO055 is being tracked by an additional LED. In the beginning of the drive and before changing to the automated drive state, the sensor should be caibrated by perorming some manuevers which can be found in sensor manuals. For this purposes, the low-level controller is used [link](https://github.com/attaoveisi/L2C_Act_AttBot2). 

## Odometry

The odometry is realized by two motor encoders and the steering wheel angle estimation. An open loop single-trach model is used for forward estimation of the 2D velocities and position. It is assummed that AttBot 2.0 is only able to move on the horizontal plane (trust me it won't fly).

two motor encoders are implemented on the front and wheel axis as it can be seen below (with the accuray of 20 ticks per one round):

![image](https://user-images.githubusercontent.com/17289954/103779090-87fd2300-5033-11eb-8899-f1b6ed48ae03.png)


