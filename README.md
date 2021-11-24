# mocca_robot
![Robot](docs/mocca_robot.png)

## Hardware

### SBC
- [Raspberry Pi](https://www.raspberrypi.org/)
- [LattePanda](https://www.lattepanda.com/)
- [ODROID-H2](https://www.hardkernel.com/shop/odroid-h2plus/)

### Motor
* Mobile: Robotis [XL430-W250-T](https://www.robotis.com/shop/item.php?it_id=902-0135-000) (2pcs)

   Speed: 1Mbps  
   ID1: Right Wheel  
   ID2: Left Wheel  
* Motion: Robotis [AX-12A](https://www.robotis.com/shop/item.php?it_id=902-0003-001) (8pcs)

   Speed: 1Mbps  
   ID11, 12, 13: Left Arm  
   ID21, 22, 23: Right Arm  
   ID31, 32: Neck  

### Interface
- [U2D2](https://www.robotis.com/shop/item.php?it_id=902-0132-000)

## Software

### Install Ubuntu
- [Ubuntu-Mate](https://ubuntu-mate.org/download/armhf/bionic/) for Raspberry Pi
- [Ubuntu18.04](https://releases.ubuntu.com/18.04/) for general PC

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

### Install Required ROS library
```
sudo apt install ros-melodic-rosbridge-server
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

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
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
rosrun mocca_robot test_motion.py
```
