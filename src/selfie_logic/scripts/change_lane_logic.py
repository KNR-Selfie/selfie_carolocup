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
  #rospy.loginfo("Points: %d \t Lane: %d", CLC.points_on_lane, CLC.right_lane)

def obstacles_callback_old(msg):
  '''
  CLC.points_on_lane = 0
  #if we are on right lane and want to detect obstacle
  if CLC.start_maneuver == 0:
    for box_nr in range (len(msg.polygons)-1, 0, -1):    
      CLC.polygon.polygon = msg.polygons[box_nr]
      #check each polygon
      CLC.check_polygon_two_lines_middle_four_points()
  
    if CLC.points_on_lane>0:
      CLC.counter +=1
      if CLC.counter > 5:
        CLC.counter = 5
    else:
      CLC.counter -=1
      if CLC.counter<0:
        CLC.counter = 0

    if CLC.counter>1:
      CLC.start_maneuver = 1
      CLC.start_distance = CLC.distance
      CLC.counter = 0
      CLC.changed = 0

  #when we are on left lane as part of maneuver
  elif CLC.start_maneuver ==1:
    #clear variable
    CLC.obstacle_on_right = 0

    #check polygons
    for box_nr in range (len(msg.polygons)-1, 0, -1):    
      CLC.polygon.polygon = msg.polygons[box_nr]
      CLC.check_polygon_maneuver()

    #if we are on right lane and there are obstacles on right
    if CLC.right_lane == 0 and CLC.obstacle_on_right > 0:
      CLC.counter +=1
      if CLC.counter > 5:
        CLC.counter = 5
    elif CLC.right_lane == 0 and CLC.obstacle_on_right ==0 and CLC.changed ==0:
      CLC.counter -=1
      if CLC.counter<0:
        CLC.counter = 0
    elif CLC.right_lane ==0 and CLC.obstacle_on_right ==0 and CLC.changed ==1:
      CLC.no_obstacles +=1
    elif CLC.right_lane ==0 and CLC.obstacle_on_right > 0 and CLC.changed ==1:
      CLC.no_obstacles -=3
      if CLC.no_obstacles < 0:
        CLC.no_obstacles = 0

    #if we have more than three decide we have changed lane
    if CLC.counter>3:
      CLC.changed = 1

    if CLC.no_obstacles > 4 and CLC.got_stop_distance = 0:
      CLC.lane_distance = CLC.distance - CLC.start_distance
      CLC.got_stop_distance = 1
    elif CLC.no_obstacles ==0:
      CLC.got_stop_distance = 0
     

    if CLC.changed == 1 and CLC.right_lane ==1:
      CLC.start_maneuver = 0'''
    
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
    
    speed_logic_pub = rospy.Publisher("speed_logic", Float32, queue_size=1)

    
    while not rospy.is_shutdown():
      '''if CLC.right_lane ==1:
        rospy.loginfo("R. Man: %d Obs: %d ", CLC.start_maneuver, CLC.obstacle_on_right)
      elif CLC.right_lane ==0:
        rospy.loginfo("L. Man: %d Obs: %d ", CLC.start_maneuver, CLC.obstacle_on_right)
      '''
      
    
      #rospy.loginfo("%d", len(CLC.polygons))

      if CLC.get_call == 0:
        CLC.polygons[:] = []
      
        

      CLC.change_lane_procedure()
      CLC.get_call = 0
      rospy.loginfo("T: %d. La: %d",CLC.trybe, CLC.right_lane)

      rospy.sleep(0.1)
      #rospy.spin()
