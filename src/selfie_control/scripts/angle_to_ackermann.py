#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.server import Server
from selfie_control.cfg import SpeedConfig

UPDATE_RATE = 50

def steering_angle_callback(msg):
    cmd = AckermannDriveStamped()

    cmd.drive.steering_angle = msg.data
    cmd.drive.speed = speed
    cmd.drive.steering_angle_velocity = 15.0

    drive_pub.publish(cmd)

def speed_callback(msg):
    global speed
    speed = msg.data

if __name__ == '__main__':
    rospy.init_node('selfie_angle_to_ackermann')

    global drive_pub
    drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)
    speed_sub = rospy.Subscriber('speed_control', Float64, speed_callback, queue_size=1)
    angle_sub = rospy.Subscriber('steering_angle', Float64, steering_angle_callback, queue_size=1)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown(): rate.sleep()
