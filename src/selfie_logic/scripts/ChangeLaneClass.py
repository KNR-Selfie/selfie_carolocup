import rospy
from geometry_msgs.msg import Polygon, PolygonStamped
import actionlib
import selfie_control.msg
import numpy as np
from math import sqrt, atan, atan2
from scipy.optimize import minimize
from numpy.polynomial.polynomial import Polynomial

class ChangeLaneClass:
    def __init__(self):
        #right_lane =1  we are on right, right_lane = 0 we are on left
        self.right_lane = 1
        
        #distance from encoders
        self.distance = 0

        #polynomial
        self.c_poly = Polynomial(0)
        self.r_poly = Polynomial(0)
        self.l_poly = Polynomial(0)

        self.lookahead_c = 0.2 

        #obstacles polygon
        self.polygons = []

        #points on right lane
        self.points_front = 0
        self.points_right = 0
        
        self.start_distance = 0
        self.change_distance = 0

        #distance from center lane
        self.center_dis = 0
        
        self.get_call = 0
        self.once_detected =0

        self.get_change_distance = 0
        self.get_start_distance =0

    def create_client(self):
        self.client = actionlib.SimpleActionClient('change_lane', selfie_control.msg.ChangeLaneAction)
        rospy.loginfo("Wait for connecting to server")
        self.client.wait_for_server()
        rospy.loginfo("Connected with server 'change_lane'")
        

    def get_offset(self, lookahead, poly):
        '''
        Method that return y coordinate of polymian on x coordinate
        lookahead - x coordinate
        poly - polymian that we want to get distance from
        '''
        x_shifted = Polynomial([-lookahead, 1])

        c_poly_dist_sq = x_shifted**2 + poly**2

        c_x = max(lookahead, minimize(c_poly_dist_sq, lookahead).x[0])
        c_y = poly(c_x)

        c_dist = sqrt(c_poly_dist_sq(c_x))

        slope = poly.deriv()(c_x)
        theta = atan(slope)

        if atan2(c_y, x_shifted(c_x)) - theta > 0:
            c_dist = -c_dist

        return c_dist
        

    def check_polygon(self, polygon):
        '''
        Method that checks if middle points of polygon are inside current lane
        '''
        tmp_front = 0
        tmp_right = 0        
        for i in range(0,4):
            x_coor = polygon.points[i].x
            y_coor = polygon.points[i].y
                
            #get y_coor on right and center line with margin
            c_dis = self.get_offset(x_coor, self.c_poly)
            r_dis = self.get_offset(x_coor, self.r_poly)
                
            if c_dis < y_coor and y_coor<r_dis:
                tmp_front +=1   

            if y_coor>0 and y_coor<r_dis and x_coor > 0.0:
                tmp_right += 1

        if tmp_front>3:
            self.points_front +=1

        if tmp_right>3:
            self.points_right  +=1  
            
        
    def check_polygons(self):
        #checking if we have obstacle in front
        self.center_dis = self.get_offset(self.lookahead_c,self.c_poly)
        if self.center_dis<0:
            self.right_lane = 1
        else:
            self.right_lane = 0

        #if we are on right lane and want to detect obstacle
        #check each polygon
        if (len(self.polygons)==0):
            return 0

        self.points_front = 0
        self.points_right = 0

        for box_nr in range (len(self.polygons)-1, 0, -1):   
            self.check_polygon(self.polygons[box_nr])

    def change_lane_procedure(self):
        #main methode to change lane

        self.check_polygons()

        #if we are on right lane and want to change it
        if self.right_lane==1 and self.points_front >0:
            if self.once_detected <2:
                self.once_detected += 1
            elif (self.get_start_distance==0):
                self.once_detected= 0 
                self.get_start_distance=1
                self.start_distance = self.distance
                goal = selfie_control.msg.ChangeLaneGoal(left_lane=True)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                #result = self.client.get_result()

        elif self.right_lane==1 and self.points_front ==0:
            self.once_detected =0

        #if we are on left lane
        elif self.right_lane==0 and self.points_right==0:
            
            if self.once_detected <2:
                self.once_detected += 1
            if (self.get_change_distance ==0):
                self.change_distance = self.distance - self.start_distance
                self.start_distance = self.distance
                self.get_change_distance = 1
            else:
                #if self.distance-self.start_distance>self.change_distance:
                    self.once_detected= 0 
                    self.get_change_distance = 0
                    self.get_start_distance =0
                    goal = selfie_control.msg.ChangeLaneGoal(left_lane=False)
                    self.client.send_goal(goal)
                    self.client.wait_for_result()
                
    
    


        