#! /usr/bin/env python3
# -+- coding: utf-8 -+-

import rospy
import actionlib
from mocca_robot.msg import MoccaMotionAction, MoccaMotionGoal
from std_msgs.msg import String
import rospkg
import os
import json
import sys

rospack = rospkg.RosPack()

def mocca_motion_client(motion_string):
    # rospy.loginfo('mocca_motion_client:' + motion_string)
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/mocca_motion', MoccaMotionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    client.cancel_all_goals()

    # Creates a goal to send to the action server.
    goal = MoccaMotionGoal(motion_data=motion_string)
    rospy.loginfo('goal:' + goal.motion_data)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    # client.wait_for_result()

    rospy.loginfo('Done with the result: ' + str(client.get_result()))

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def playMotionRaw(motion_raw):
    # rospy.loginfo('playMotionRaw %s', motion_raw)
    result = mocca_motion_client(motion_raw)
    rospy.loginfo("Result:" + str(result))

def playMotionName(motion_name):
    rospy.loginfo('playMotionName %s', motion_name)
    path = rospack.get_path('mocca_robot') + '/motions'
    motionfiles = os.listdir(path)
    for motionfile in motionfiles:
        # rospy.loginfo('MotionFile is :' + motionfile.lower())
        # rospy.loginfo('MotionName is :' + motion_name.lower())
        if motionfile.lower().find(motion_name.lower()) >= 0:
            f = open(path + '/' + motionfile, mode='r')
            motion_raw = f.read()
            playMotionRaw(motion_raw)
            rospy.loginfo(f.read())
            f.close()
            break

def playMotion(motion_json):
    rospy.loginfo('playMotion ' + motion_json.data)
    loaded_json = json.loads(motion_json.data.encode().decode('unicode-escape')
    for x in loaded_json:
        # rospy.loginfo('playMotion %s %s', x, loaded_json[x])
        if x == 'name':
            playMotionName(loaded_json[x])
        elif x == 'raw':
            playMotionRaw(loaded_json[x])
    rospy.loginfo('playMotion Done')


if __name__ == '__main__':
    try:
        rospy.init_node('mocca_motion_play_by_filename')

        subMotion = rospy.Subscriber('mocca_motion_raw', String, playMotion)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion", sys.stderr)
