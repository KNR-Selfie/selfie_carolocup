#!/usr/bin/env python

import rospy

import numpy as np
from scipy.optimize import minimize
from numpy.polynomial.polynomial import Polynomial

from selfie_msgs.msg import RoadMarkings
from std_msgs.msg import Float64, Float32, UInt16
from selfie_msgs.msg import PolygonArray
from ChangeLaneClass import ChangeLaneClass
from geometry_msgs.msg import Polygon, PolygonStamped

CLC= ChangeLaneClass()


def road_markings_callback(msg):
  #save parameters for each line
  CLC.c_poly = Polynomial(msg.center_line)
  CLC.r_poly = Polynomial(msg.right_line)
  CLC.l_poly = Polynomial(msg.left_line)

def distance_callback(msg):
  #save distance from stm32
  CLC.distance = msg.data
  #showing variables on screen
  #rospy.loginfo("Points: %d \t Lane: %d", CLC.points_on_lane, CLC.right_lane)
  
def obstacles_callback(msg):
  if (CLC.get_call ==0):
    CLC.polygons[:] = []
    CLC.get_call = 1
    for box_nr in range (len(msg.polygons)-1, 0, -1):    
        CLC.polygons.append(msg.polygons[box_nr])

    
if __name__ == '__main__':
    rospy.init_node('change_lane_logic')

    road_markings_sub = rospy.Subscriber('vision/road_markings', RoadMarkings, road_markings_callback, queue_size=1)
    road_markings_sub = rospy.Subscriber('obstacles', PolygonArray, obstacles_callback, queue_size=1)
    road_markings_sub = rospy.Subscriber('stm32/distance', Float32, distance_callback, queue_size=1)
   
    change_lane_pub = rospy.Publisher('change_lane_status', UInt16, queue_size=1)
   
    CLC.border_distance_x = rospy.get_param('~border_x')
    CLC.border_distance_y = rospy.get_param('~border_y')
    CLC.fraction = rospy.get_param('~fraction', 1.0)
    rospy.loginfo("border_x = %f, border_y = %f, fraction = %f",CLC.border_distance_x, CLC.border_distance_y, CLC.fraction)

    CLC.create_client()
    
    while not rospy.is_shutdown():
      if CLC.get_call == 0:
        CLC.polygons[:] = []

            
      #rospy.loginfo("Lane: %d F: %d R: %d",CLC.right_lane, CLC.points_front, CLC.points_right)

      CLC.change_lane_procedure()
      CLC.get_call = 0
      #rospy.loginfo("T: %d. La: %d",CLC.trybe, CLC.right_lane)
      change_lane_pub.publish(CLC.change_lane_status)
      rospy.sleep(0.1)
      #rospy.spin()
