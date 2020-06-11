# mocca_robot

## Requirement
- DynamixelSDK: This project require DynamixelSDK for robot
```
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd DynamixelSDK/python
$ sudo python setup.py install
```
- PAHO-MQTT: This project require mqtt client for getting motion command
```
$ pip install paho-mqtt
```

## Display mocca robot on Rviz
```
$ roslaunch mocca_robot display.launch
```

## Standby mocca robot (It use /dev/ttyUSB0 for communicate with Dynamixel)
```
$ roslaunch mocca_robot bringup.launch
```

## Test motion
```
$ roslaunch mocca_robot test_motion.py
```
