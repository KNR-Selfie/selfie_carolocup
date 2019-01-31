#!/usr/bin/env python

import rospy

import numpy as np
from scipy.optimize import minimize
from numpy.polynomial.polynomial import Polynomial

from selfie_msgs.msg import RoadMarkings
from std_msgs.msg import Float64, Float32
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
  rospy.loginfo("Points: %d \t Lane: %d", CLC.points_on_lane, CLC.right_lane)

def obstacles_callback(msg):
  CLC.points_on_lane = 0

  for box_nr in range (len(msg.polygons)-1, 0, -1):    
    CLC.polygon.polygon = msg.polygons[box_nr]
    #check each polygon
    CLC.check_polygon()
  
  
if __name__ == '__main__':
    rospy.init_node('change_lane_logic')

    road_markings_sub = rospy.Subscriber('vision/road_markings', RoadMarkings, road_markings_callback, queue_size=1)
    road_markings_sub = rospy.Subscriber('obstacles', PolygonArray, obstacles_callback, queue_size=1)
    road_markings_sub = rospy.Subscriber('stm32/distance', Float32, distance_callback, queue_size=1)
    
    speed_logic_pub = rospy.Publisher("speed_logic", Float32, queue_size=1)

    rate = rospy.Rate(50)
  
    
    while not rospy.is_shutdown():
      #rospy.loginfo("Points: %d \t Lane: %d", CLC.points_on_lane, CLC.right_lane)
      #rospy.loginfo(CLC.distance, CLC.right_lane)
      #rate.sleep()
      rospy.spin()
