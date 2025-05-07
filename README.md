# BalanceBoyV2
My private and better version of my master-thesis

## Course Description (more to come)
Due to  non-existent budget i was not able to get the first version of this self-balancing robot working during my master-thesis.
The BLDC-motors my university provided were way to powerful to control the robot in a way that it balances by itself.
Therefore I had to switch to normal DC-Motors....
But now, 2 years later I managed to build a new version of the BalanceBoy using BLDC-Motors.
I'm doing this in my spare-time, so progress will be slow


Here are the main facts about the hardware:
* 2 BLDC-Motors: iPower GM5208-24
* 2 Magnetic Encoders: AS5600
* 2 BLDC-Drivers: B-G341B-ESC1
* Raspberry PI Zero 2
* 9-DOF IMU: BNO085
* 2 M5-NeoHex as "eyes"
And here the current functionalities:
* Main-Controller: Python (on RPi)
  * Control-Library:
    * LQG-Control for balancing: LQR-Controller + Kalman Estimator
    * PID-Control for yaw
  * Using ZMQ for
    * changing weight-matrices of the LQR-Controller and Kalman-Estimator (the gains are then calculated on the robot)
    * changing PID-Gains for yaw-control
    * changing setpoints
    * motor identification
    * plotting data
  * Using FastAPI to provide a webserver for live sensor-charts on the laptop/phone
  * Using evdev to connect to a PS4 DualsShock-Controller
* Motor-Controller:
  * SimpleFOC
    * capable using FOC-Alghorithm, but using voltage for controlling tourque
    * Drivers provide current-sensing, but it's shitty

I'll provide the code for motors and some photos and videos soon...

Cheers!
