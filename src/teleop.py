#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs import Twist
from std_msgs.msg import Joy
from threading import Thread, Timer

class MoccaTeleop(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber('joy', Joy, joyCallback, 10)

        rospy.loginfo("Mocca robot initialized")

    def run(self):
        rospy.loginfo('run')
        self._run = True
        while self._run:
            rospy.sleep(0.01)

        rospy.loginfo('stopped')

    def stop(self):
        rospy.loginfo('stop')
        self._run = False

    def publish(self):

    def joyCallback(self, joy):
        Twist vel
        vel.angular.z = joy.axes[0]


if __name__ == '__main__':
    try:
        rospy.init_node('mocca_teleop')

        server = MoccaRobot()
        server.start()
        while not rospy.is_shutdown():
            rospy.sleep(0.05)
        server.stop()
    except rospy.ROSInterruptException:
        server.stop()
        rospy.logerr("program interrupted before completion", sys.stderr)