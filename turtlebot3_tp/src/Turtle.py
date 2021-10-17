#!/usr/bin/env python
# Created by: Jesse Sunkur
# TP: SMA

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from scipy.ndimage.filters import uniform_filter1d

def callback(tb1):
    global TB1_pose_x
    global TB1_pose_y
    global TB1_pose_z
    global TB1_orientation_x
    global TB1_orientation_y
    global TB1_orientation_z

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    TB1_pose_x = (tb1.pose.pose.position.x)
    TB1_pose_y = (tb1.pose.pose.position.y)
    TB1_pose_z = (tb1.pose.pose.position.z)
    TB1_orientation_x = (tb1.pose.pose.orientation.x)
    TB1_orientation_y = (tb1.pose.pose.orientation.y)
    TB1_orientation_z = (tb1.pose.pose.orientation.z)
    TB1_orientation_w = (tb1.pose.pose.orientation.w)

    #roll = euler_from_quaternion (TB2_orientation_x)
    #pitch = euler_from_quaternion (TB2_orientation_y)
    #yaw = euler_from_quaternion (TB2_orientation_z)
    #print("Position TB1")
    #print("x: ", TB1_pose_x)
    #print("y: ", TB1_pose_y)
    #print("z: ", TB1_pose_z)
    #print("Orientation TB1")
    #print("x: ", TB1_orientation_x)
    #print("y: ", TB1_orientation_y)
    #print("z: ", TB1_orientation_z)
    #print("w: ", TB1_orientation_w)

    quat = quaternion_from_euler (roll, pitch,yaw)

    rospy.Subscriber("/TB2/amcl_pose", PoseWithCovarianceStamped, callback1)

def callback1(tb2):
    global TB2_pose_x
    global TB2_pose_y
    global TB2_pose_z
    global TB2_orientation_x
    global TB2_orientation_y
    global TB2_orientation_z
    global yaw
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    TB2_pose_x = (tb2.pose.pose.position.x)
    TB2_pose_y = (tb2.pose.pose.position.y)
    TB2_pose_z = (tb2.pose.pose.position.z)
    TB2_orientation_x = (tb2.pose.pose.orientation.x)
    TB2_orientation_y = (tb2.pose.pose.orientation.y)
    TB2_orientation_z = (tb2.pose.pose.orientation.z)
    TB2_orientation_w = (tb2.pose.pose.orientation.w)

    #roll = euler_from_quaternion (TB2_orientation_x)
    #pitch = euler_from_quaternion (TB2_orientation_y)
    #yaw = euler_from_quaternion (TB2_orientation_z)
    #print("Position TB2")
    #print("x: ", TB2_pose_x)
    #print("y: ", TB2_pose_y)
    #print("z: ", TB2_pose_z)
    #print("Orientation TB2")
    #print("x: ", TB2_orientation_x)
    #print("y: ", TB2_orientation_y)
    #print("z: ", TB2_orientation_z)
    #print("w: ", TB2_orientation_w)

    quat = quaternion_from_euler (roll, pitch,yaw)

    rospy.Subscriber("/TB3/amcl_pose", PoseWithCovarianceStamped, callback2)

def callback2(tb3):
    global TB3_pose_x
    global TB3_pose_y
    global TB3_pose_z
    global TB3_orientation_x
    global TB3_orientation_y
    global TB3_orientation_z

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    TB3_pose_x = (tb3.pose.pose.position.x)
    TB3_pose_y = (tb3.pose.pose.position.y)
    TB3_pose_z = (tb3.pose.pose.position.z)
    TB3_orientation_x = (tb3.pose.pose.orientation.x)
    TB3_orientation_y = (tb3.pose.pose.orientation.y)
    TB3_orientation_z = (tb3.pose.pose.orientation.z)
    TB3_orientation_w = (tb3.pose.pose.orientation.w)

    #roll = euler_from_quaternion (TB2_orientation_x)
    #pitch = euler_from_quaternion (TB2_orientation_y)
    #yaw = euler_from_quaternion (TB2_orientation_z)
    #print("Position TB3")
    #print("x: ", TB3_pose_x)
    #print("y: ", TB3_pose_y)
    #print("z: ", TB3_pose_z)
    #print("Orientation TB3")
    #print("x: ", TB3_orientation_x)
    #print("y: ", TB3_orientation_y)
    #print("z: ", TB3_orientation_z)
    #print("w: ", TB3_orientation_w)

    yaw1 = math.degrees(yaw)

    quat = quaternion_from_euler (roll, pitch,yaw)
    func()

def func():

        tb1 = np.array((TB1_pose_x, TB1_pose_y))
        tb2 = np.array((TB2_pose_x, TB2_pose_y))
        dist = np.linalg.norm(tb1-tb2)

        tb1 = np.array((TB1_pose_x, TB1_pose_y))
        tb3 = np.array((TB3_pose_x, TB3_pose_y))
        dist1 = np.linalg.norm(tb1-tb3)

        print("TB1_x: ", round(TB1_pose_x,1), "TB1_y: ", round(TB1_pose_y,1))

        if int(dist > 0.5):
            movebase()

        if int(dist1 > 0.5):
            movebase1()

        if int(dist <= 1.5):
            pass

        if int(dist1 <= 1.5):
            pass

def movebase():
    client = actionlib.SimpleActionClient('TB2/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = round(TB1_pose_x, 1)
    goal.target_pose.pose.position.y = round(TB1_pose_y, 1)
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    success = client.wait_for_result()

def movebase1():
    client = actionlib.SimpleActionClient('TB3/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = round(TB1_pose_x, 1)
    goal.target_pose.pose.position.y = round(TB1_pose_y, 1)
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    success = client.wait_for_result()


if __name__=="__main__":
    rospy.init_node("turtle_leader",anonymous=True)
    rospy.Subscriber("/TB1/amcl_pose", PoseWithCovarianceStamped, callback)

    rospy.Subscriber("/TB2/amcl_pose", PoseWithCovarianceStamped, callback1)

    rospy.Subscriber("/TB3/amcl_pose", PoseWithCovarianceStamped, callback2)

    rospy.spin()
