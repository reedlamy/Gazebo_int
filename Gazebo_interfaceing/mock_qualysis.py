#!/usr/bin/python3
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
import math
import time
from datetime import datetime
from geometry_msgs.msg import PoseStamped, Pose


def callback(data):
    global ext_x, ext_y, ext_z, ext_rotx, ext_roty, ext_rotz, ext_rotw
    # Quaternians

    # Reset current external position variables
    ext_x = data.position.x
    ext_y = data.position.y
    ext_z = data.position.z
    ext_rotx = data.orientation.x
    ext_roty = data.orientation.y
    ext_rotz = data.orientation.z
    ext_rotw = data.orientation.w

# Main function to run until node is killed


    # Start streaming frames #####################################################################################################################################3
    #await connection.stream_frames(components=["6deuler"], on_packet=on_packet)

    # Wait asynchronously until shutdown

    # Stop streaming################################################################################################################################
    #await connection.stream_frames_stop()


# Function to run when script is run as main script
if __name__ == "__main__":

    # Initialize ROS node for Qualisys stream with worldframe and IP address
    rospy.init_node('qualisys_cf_stream', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    firstPacket = True
    start_record = False

    # Automatically compiles Crazyflie rigid body names from launch file into array
    # Note: Names must be equal to names in Qualisys (normally CF#)
    cf_names = rospy.get_param("/cf_names")
    cf_names = cf_names.split(',')

    time.sleep(5) #wait for gazebo to finish loading

    for i in range(len(cf_names)):
        rospy.Subscriber("/firefly/odometry_sensor1/pose", Pose, callback)
        # Assign world position components to msg (after converting to meters)
        try:
            rospy.wait_for_message("/firefly/odometry_sensor1/pose", Pose, timeout=1)
        except rospy.ROSException:
            print("/external_position")
            #rospy.logerr('Could not subscribe to ' + name + '/external_position message: Timeout')
            rospy.logerr('Could not subscribe to  /external_pose message: Timeout1')
            sys.exit()
        except rospy.ROSInterruptException:
            rospy.logerr('USER INTERRUPTION')
            sys.exit()

    #log_location = '/home/reed/Crazyfly_ws/src/NASLab_Crazyflies/crazyflie_scripts/static_cf_actual_pos_logs/'

    now = datetime.now()
    dt_string = now.strftime("%m-%d-%Y_%H:%M")

    csv_xheader = [name + ' X' for name in cf_names]
    csv_yheader = [name + ' Y' for name in cf_names]
    csv_header = [None]*(len(csv_xheader)+len(csv_yheader))
    csv_header[::2] = csv_xheader
    csv_header[1::2] = csv_yheader
    #with open(log_location + 'cf_xy_pos_' + dt_string + '.csv', 'w', newline='') as file:
        #writer = csv.writer(file, quoting=csv.QUOTE_NONNUMERIC)
        #writer.writerow(csv_header)
    rate = rospy.Rate(10)

    wanted_bodies = cf_names
    publishers = []
    # Iterate over all CFs in launch file
    for j in range(len(wanted_bodies)):
        # PoseStamped

        # Publisher name preceded by 'CF#/' due to grouping in launch file
        pub = rospy.Publisher(wanted_bodies[j] + "/external_pose", PoseStamped, queue_size=1)
        publishers.append(pub)  # Array containing a publisher for each Crazyflie


    while not rospy.is_shutdown():


        # Initialize messages & publishers to 'external_position' topic (subscribed by Crazyflie)


        # Iterate over all CFs in launch file
        for j in range(len(wanted_bodies)):
            # PoseStamped
            pos_msg = PoseStamped()
            pos_msg.header.frame_id = worldFrame
            pos_msg.header.stamp = rospy.Time.now()
            pos_msg.header.seq += 1
            pos_msg.pose.position.x = ext_x
            pos_msg.pose.position.y = ext_y
            pos_msg.pose.position.z = ext_z
            pos_msg.pose.orientation.x = ext_rotx
            pos_msg.pose.orientation.y = ext_roty
            pos_msg.pose.orientation.z = ext_rotz
            pos_msg.pose.orientation.w = ext_rotw

            #messages.append(pos_msg)  # Array containing a message for each Crazyflie

            publishers[j].publish(pos_msg)



        rate.sleep()
