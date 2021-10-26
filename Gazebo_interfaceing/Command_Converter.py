#!/usr/bin/python
# Author: Chris Moneyron, Purdue University, cmoneyron@gmail.com
# Professor: Nina Mahmoudian, Purdue University, ninam@purdue.edu
#
# Streaming 6Dof from QTM
# Make sure that new QTM Measurement is open before running
#
# This code is adapted from the code provided by Qualisys for 6DOF tracking
# Link: https://github.com/qualisys/qualisys_python_sdk/blob/master/examples/stream_6dof_example.py

import xml.etree.ElementTree as ET
import rospy
import sys
import csv
import tf.transformations
import math
import time
from datetime import datetime
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from crazyflie_driver.msg import Position

def callback(data):
    global des_x,des_y,des_z,des_yaw
    # Quaternians

    # Reset current external position variables
    des_x=data.x
    des_y=data.y
    des_z=data.z
    des_yaw=data.yaw

def msg_converter(x,y,z,yaw):

    # create trajectory msg
    traj = MultiDOFJointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = 'frame'
    traj.joint_names.append('base_link')

    # create start point for trajectory
    #transforms = Transform()
    #velocities = Twist()
    #accel = Twist()
    #point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accel], rospy.Time(1))
    #traj.points.append(point)

    # create end point for trajectory
    transforms = Transform()
    transforms.translation.x = x
    transforms.translation.y = y
    transforms.translation.z = z

    quat = tf.transformations.quaternion_from_euler(yaw * np.pi / 180.0, 0, 0, axes='rzyx')
    transforms.rotation.x = quat[0]
    transforms.rotation.y = quat[1]
    transforms.rotation.z = quat[2]
    transforms.rotation.w = quat[3]

    velocities = Twist()
    accel = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accel], rospy.Time(2))
    traj.points.append(point)

    return traj

# Function to run when script is run as main script
if __name__ == "__main__":

    # Initialize ROS node for Qualisys stream with worldframe and IP address
    rospy.init_node('Command_Converter', anonymous=True)

    des_x=0
    des_y=0
    des_z=0
    des_yaw=0

    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    time.sleep(5) #wait for gazebo to finish loading

    for i in range(len(cf_names)):
        rospy.Subscriber("/CF2/cmd_position", Position, callback)
        # Assign world position components to msg (after converting to meters)
        try:
            rospy.wait_for_message("/CF2/cmd_position", Position, timeout=1)
        except rospy.ROSException:
            print("/external_position")
            #rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
            rospy.logerr('Could not subscribe to  /external_pose message: Timeout2')
            sys.exit()
        except rospy.ROSInterruptException:
            rospy.logerr('USER INTERRUPTION')
            sys.exit()


    rate = rospy.Rate(10)

    wanted_bodies = cf_names
    publishers = []
    # Iterate over all CFs in launch file
    for j in range(len(wanted_bodies)):
        # PoseStamped

        # Publisher name preceded by 'CF#/' due to grouping in launch file
        pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
        publishers.append(pub)  # Array containing a publisher for each Crazyflie



    while not rospy.is_shutdown():


        # Initialize messages & publishers to 'external_position' topic (subscribed by Crazyflie)

        # Iterate over all CFs in launch file
        for j in range(len(wanted_bodies)):
            # PoseStamped
            pos_msg = msg_converter(des_x,des_y,des_z,des_yaw)
            publishers[j].publish(pos_msg)
            rate.sleep()

