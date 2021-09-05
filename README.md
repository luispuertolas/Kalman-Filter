#kalman filter
#Coded by Luis Andrés Puértolas Bálint


Interfacing the ADXL345 accelerometer and the IMU_ITG3200 gyroscope on an Arduino board.

Content:

IMU.ino - Arduino code with serial connection to the computer, and wired to the IMU.

Fusion Kalman Filter.cs - A faster, less accurate filter I found published.

Kalman final.cs - Discrete Kalman Filter, alongside works with IMU.ino, and eliminates the noise of the IMU_ITG3200 gyroscope to unnoticeable levels. The code filters position, speed and acceleration in the X, Y and X axes, and the IMU's temperature.

RightForeArmController.cs - Controlls a forearm in Unity with an IMU. This code needs to be attached to a forearm component of a 3D avatar.
