# mocca_robot

## Hardware

### SBC
- [Raspberry Pi](https://www.raspberrypi.org/)
- [LattePanda](https://www.lattepanda.com/)
- [ODROID-H2](https://www.hardkernel.com/shop/odroid-h2plus/)

### Motor
- Mobile: Robotis [XL430-W250-T](https://www.robotis.com/shop/item.php?it_id=902-0135-000) (2pcs)
- Motion: Robotis [AX-12A](https://www.robotis.com/shop/item.php?it_id=902-0003-001) (8pcs)


## Software

### Install Ubuntu
- [Ubuntu18.04](https://releases.ubuntu.com/18.04/) for general PC
- [Ubuntu-Mate](https://ubuntu-mate.org/ports/raspberry-pi/) for Raspberry Pi

### [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) (Recommended melodic)

### [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python
sudo python setup.py install
```
- Grant tty access to user
```
sudo usermod -aG dialout $USER
sudo reboot now
```

### PAHO-MQTT: This project require mqtt client for getting command
```
pip install paho-mqtt
```

## Build
- Download project
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/movehoon/mocca_robot.git
cd ~/catkin_ws
```

- Build
```
catkin_make
```

## Display mocca robot on Rviz
```
roslaunch mocca_robot display.launch
```

## Standby mocca robot (It use /dev/ttyUSB0 for communicate with Dynamixel)
```
roslaunch mocca_robot bringup.launch
```

## Test motion
```
roslaunch mocca_robot test_motion.py
```
