#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8, Int16MultiArray
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from threading import Thread, Timer
import json
import time
import math

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_MOVING_SPEED       = 32
ADDR_AX_PRESENT_POSITION   = 36

ADDR_XL_TORQUE_ENABLE      = 64
ADDR_XL_GOAL_VELOCITY      = 104

# Protocol version
PROTOCOL_VERSION_1         = 1.0               # See which protocol version is used in the Dynamixel
PROTOCOL_VERSION_2         = 2.0

# Default setting
# DXL_ID                      = 11                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque


class MoccaRobot(Thread):

    def dxlDegToPos(self, degree):
        position = 512 + (degree*3.45)
        return int(position)

    def dxlPosToDeg(self, position):
        degree = (position-512) * 0.29
        return float(degree)

    def dxlRpmToSpd(self, rpm):
        speed = rpm / 0.111
        return int(speed)

    def dxlInit(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler1 = PacketHandler(PROTOCOL_VERSION_1)
        self.packetHandler2 = PacketHandler(PROTOCOL_VERSION_2)
        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler1, ADDR_AX_TORQUE_ENABLE, 1)
        self.groupSyncWriteGoalPosition = GroupSyncWrite(self.portHandler, self.packetHandler1, ADDR_AX_GOAL_POSITION, 2)
        self.groupSyncWriteMovingSpeed = GroupSyncWrite(self.portHandler, self.packetHandler1, ADDR_AX_MOVING_SPEED, 2)

        # Open port
        try :
            if self.portHandler.openPort():
                rospy.loginfo("Succeeded to open the port")
            else:
                rospy.logerr("Failed to open the port %s", DEVICENAME)
                return
        except:
            rospy.logerr("Failed to open the port %s", DEVICENAME)
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            rospy.loginfo("Succeeded to change the baudrate")
        else:
            rospy.logerr("Failed to change the baudrate %s", BAUDRATE)
            return

    def dxl1Ping(self, dxl_id):
        # Ping, Check Dynamixel Connected
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler1.ping(self.portHandler, dxl_id)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler1.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("[ID:%d] ping Succeed. Dynamixel model number: %d", dxl_id, dxl_model_number)

    def dxl2Ping(self, dxl_id):
        # Ping, Check Dynamixel Connected
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler2.ping(self.portHandler, dxl_id)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler2.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("[ID:%d] ping Succeed. Dynamixel model number: %d", dxl_id, dxl_model_number)

    def torqueMotion(self, enable):
        for id in self.DXL_ID:
            self.dxl1Torque(id, enable)

    def torqueMobility(self, enable):
        self.dxl2Torque(1, enable)
        self.dxl2Torque(2, enable)

    def torqueAll(self, enable):
        # rospy.loginfo("dxlTorqueAll: %d", enable.data)
        self.torqueMotion(enable.data)
        self.torqueMobility(enable.data)

    def dxl1Torque(self, dxl_id, enable):
        # Enable Dynamixel Torque
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        dxl_comm_result, dxl_error = self.packetHandler1.write1ByteTxRx(self.portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler1.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("[ID:%d] Torque %d", dxl_id, enable)
        self.dxl_in_use = False


    def dxl1Position(self, dxl_id, position):
        # Write goal position
        # print('[dxlPosition]dxl_id:', dxl_id, 'position:', position)
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        dxl_comm_result, dxl_error = self.packetHandler1.write2ByteTxRx(self.portHandler, dxl_id, ADDR_AX_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s on ID:%d", self.packetHandler1.getTxRxResult(dxl_comm_result), dxl_id)
        elif dxl_error != 0:
            rospy.loginfo("%s", self.packetHandler1.getRxPacketError(dxl_error))
        self.dxl_in_use = False


    def dxl1GoalPositions(self, positions):
        # rospy.loginfo(' '.join(positions))
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        for i in range(len(positions)):
            param_sync_write = [DXL_LOBYTE(positions[i]), DXL_HIBYTE(positions[i])]
            dxl_addparam_result = self.groupSyncWriteGoalPosition.addParam(self.DXL_ID[i], param_sync_write)
            if dxl_addparam_result != True:
                rospy.logerr('[ID%d] gruptSyncWrite addparam failed', self.DXL_ID[i])
                break
        dxl_comm_result = self.groupSyncWriteGoalPosition.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr('[dxlGoalPositions] %s', self.packetHandler1.getTxRxResult(dxl_comm_result))

        self.groupSyncWriteGoalPosition.clearParam()
        self.dxl_in_use = False

    def dxl1MovingSpeed(self, speeds):
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        for i in range(len(speeds)):
            if (speeds[i] > 1023):
                speeds[i] = 1023
            param_sync_write = [DXL_LOBYTE(speeds[i]), DXL_HIBYTE(speeds[i])]
            dxl_addparam_result = self.groupSyncWriteMovingSpeed.addParam(self.DXL_ID[i], param_sync_write)
            if dxl_addparam_result != True:
                rospy.logerr('[ID%d] gruptSyncWrite addparam failed', self.DXL_ID[i])
                break
        dxl_comm_result = self.groupSyncWriteMovingSpeed.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr('[dxl1MovingSpeed] %s', self.packetHandler1.getTxRxResult(dxl_comm_result))

        self.groupSyncWriteMovingSpeed.clearParam()
        self.dxl_in_use = False

    def goalPose(self, pose):
        eta = float(pose.layout.data_offset) / 1000
        rospy.loginfo("goalFrame to %s %d(%f)", str(pose.data), pose.layout.data_offset, eta)

        # calculate moving speed
        for i in range(len(pose.data)):
            targetAngle = pose.data[i]
            presentAngle = self.dxlPosToDeg(self.presentPosition[i])
            moveToAngle = math.fabs(targetAngle - presentAngle)
            if eta != 0:
                self.movingSpeed[i] = int(moveToAngle / eta * 60 / 60)
            else:
                self.movingSpeed[i] = 0
            # rospy.loginfo("[%d]%f %f = %f => (%d)", i, targetAngle, presentAngle, moveToAngle, self.movingSpeed[i])

        self.dxl1MovingSpeed(self.movingSpeed)
        for i in range(len(pose.data)):
            self.goalPosition[i] = self.dxlDegToPos(pose.data[i] * self.DXL_DR[i])
        rospy.loginfo("goalPosition to %s", str(self.goalPosition))
        self.dxl1GoalPositions(self.goalPosition)


    def goalAngle(self, angles):
#        rospy.loginfo("goalAngle to %s", str(angles))
        self.dxl1MovingSpeed(self.movingSpeed)
        for i in range(len(angles.data)):
            self.goalPosition[i] = self.dxlDegToPos(angles.data[i]) * self.DXL_DR[i]
        rospy.loginfo("goalPosition to %s", str(self.goalPosition))
        self.dxl1GoalPositions(self.goalPosition)

    def dxl1PresentPosition(self, dxl_id):
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler1.read2ByteTxRx(self.portHandler, dxl_id, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("[ID:%d]%s", dxl_id,self.packetHandler1.getRxPacketError(dxl_error))
        self.dxl_in_use = False
        return dxl_present_position

    def updatePresentPosition(self):
        # rospy.loginfo('updatePresentPosition')
        for i in range(len(self.DXL_ID)):
            pos = self.dxl1PresentPosition(self.DXL_ID[i])
            self.presentPosition[i] = self.dxlPosToDeg(pos)



    def dxl2Torque(self, dxl_id, enable):
        # Enable Dynamixel Torque
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        dxl_comm_result, dxl_error = self.packetHandler2.write1ByteTxRx(self.portHandler, dxl_id, ADDR_XL_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("[ID:%d]%s", dxl_id, self.packetHandler2.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("[ID:%d] Torque %d", dxl_id, enable)
        self.dxl_in_use = False

    def dxl2GoalVelocity(self, dxl_id, velocity):
        while self.dxl_in_use:
            continue
        self.dxl_in_use = True
        dxl_comm_result, dxl_error = self.packetHandler2.write4ByteTxRx(self.portHandler, dxl_id, ADDR_XL_GOAL_VELOCITY, velocity)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s on ID:%d", self.packetHandler2.getTxRxResult(dxl_comm_result), dxl_id)
        elif dxl_error != 0:
            rospy.loginfo("%s", self.packetHandler2.getRxPacketError(dxl_error))
        self.dxl_in_use = False

    def mobilityMove(self, linear_vel, angular_vel, hold=False):
        if hold:
            self.mobilityCalledTime = None
        else:
            self.mobilityCalledTime = time.time()
        l_vel = (-linear_vel - angular_vel) * 511.0
        r_vel = (linear_vel - angular_vel) * 511.0
        self.dxl2GoalVelocity(1, int(l_vel))
        self.dxl2GoalVelocity(2, int(r_vel))


    def mobility(self, twist):
        self.mobilityMove(twist.linear.x, twist.angular.z)


    def __init__(self):
        self.dxl_in_use = False

        self.DXL_ID = [11, 12, 13, 21, 22, 23, 31, 32]
        self.DXL_DR = [1,  1,  1, -1,  1,  1,  -1, -1]

        self.presentPosition = [0, 0, 0, 0, 0, 0, 0, 0]
        self.goalPosition = [512, 512, 512, 512, 512, 512, 512, 512]
        self.movingSpeed = [100, 100, 100, 100, 100, 100, 100, 100]

        self.dxlInit()
        for id in self.DXL_ID:
            self.dxl1Ping(id)
        self.dxl2Ping(1)
        self.dxl2Ping(2)

        self.torqueMotion(True)
        self.torqueMobility(True)

        self.mobilityMove(0.0, 0.0)

        self.subTorque = rospy.Subscriber('mocca_robot_torque', Int8, self.torqueAll, queue_size=10)
        self.subGoalPose = rospy.Subscriber('mocca_robot_goal_pose', Float32MultiArray, self.goalPose, queue_size=10)
        self.subGoalAngle = rospy.Subscriber('mocca_robot_goal_angle', Float32MultiArray, self.goalAngle, queue_size=10)

        self.subCmdVel = rospy.Subscriber('cmd_vel', Twist, self.mobility, queue_size=1)

        Thread.__init__(self)

        rospy.loginfo("Mocca robot initialized")

    def run(self):
        rospy.loginfo('run')
        self._run = True
        while self._run:

            #if self.mobilityCalledTime != None:
            #    if (time.time() - self.mobilityCalledTime) > 2:
            #        self.mobilityMove(0, 0)
            # for i in range(len(self.goalPosition)):
            #     self.goalPosition[i] = goal
            # self.dxl1GoalPositions(self.goalPosition)
            self.updatePresentPosition()
#            rospy.loginfo('pre: ' + ' '.join(map(str, self.presentPosition)))

            rospy.sleep(0.01)
        self.mobilityMove(0, 0)
        rospy.loginfo('stopped')

    def stop(self):
        rospy.loginfo('stop')
        self._run = False


if __name__ == '__main__':
    try:
        rospy.init_node('mocca_robot')
        pub = rospy.Publisher('mocca_robot_present_pose', Float32MultiArray, queue_size=10)
        pub_msg = Float32MultiArray()
        server = MoccaRobot()
        server.start()
        while not rospy.is_shutdown():
            pub_msg.data = server.presentPosition
            pub.publish(pub_msg)
            rospy.sleep(0.05)
        server.stop()
    except rospy.ROSInterruptException:
        server.stop()
        pass
