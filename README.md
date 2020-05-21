# mocca_robot

## Convert xacro to urdf
```
$ roscd mocca_robot
$ cd urdf
$ rosrun xacro xacro mocca.xacro > mocca.urdf
```

## Display mocca robot on Rviz
```
$ roslaunch mocca_robot display.launch
```

## Standby mocca robot
```
$ roslaunch mocca_robot bringup.launch
```

## Connect AX12 motor for motion
```
$ rosrun mocca_robot robot.py
```

## Start motion server
```
$ rosrun mocca_robot motion.py
```