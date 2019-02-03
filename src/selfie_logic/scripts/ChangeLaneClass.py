import rospy
from geometry_msgs.msg import Polygon, PolygonStamped

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
        self.lookahead_f = 0.3
        self.lookahead_max = 0.8
        #width of road
        self.width = 0.45

        #obstacles polygon
        self.polygons = []

        #points on current lane
        self.points_on_lane = 0
        self.front_points_counter = 0

        self.start_maneuver = 0
        self.start_distance = 0
        self.lane_distance = 0
        self.obstacle_on_right = 0
        self.changed_lane = 0
        self.no_obstacles = 0
        self.got_stop_distance = 0

        self.center_dis = 0
        
        self.trybe = 0
        self.get_call = 0

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
        

    def check_polygon_two_lines_middle_four_points(self, polygon):
        '''
        Method that checks if middle points of polygon are inside current lane
        '''
        points_tmp = 0
        #we are on right so we check right lane
        if self.right_lane == 1:
            x_middle =0
            y_middle = 0
            for i in range(0,4):
                #get x_coor with shift
                x_coor = polygon.points[i].x + 0.2
                #get y_coor with minus sign
                y_coor = -polygon.points[i].y

                x_middle +=x_coor
                y_middle +=y_coor

            x_middle /=4
            y_middle /=4

            x_min = min(polygon.points[0].x + 0.2,polygon.points[1].x + 0.2,polygon.points[2].x + 0.2,polygon.points[3].x + 0.2)
            y_min = min(-polygon.points[0].y,-polygon.points[1].y,-polygon.points[2].y,-polygon.points[3].y)
            y_max = max(-polygon.points[0].y,-polygon.points[1].y,-polygon.points[2].y,-polygon.points[3].y)
            y_wid = (y_max-y_min)/2

            #get y_coor on right and center line with margin
            c_dis_m = self.get_offset(x_middle, self.c_poly)
            r_dis_m = self.get_offset(x_middle, self.r_poly)
                            
            if c_dis_m<y_middle and y_middle<r_dis_m and x_min>0.3 and x_min<0.7:
                points_tmp +=1   
            #rospy.loginfo("C: %f Y: %f R: %f X:%f", c_dis_m, y_middle, r_dis_m,x_min)                 
        
        #return box only if more points are inside
        if points_tmp >0:
            self.points_on_lane += points_tmp

    def check_polygon_maneuver(self, polygon):
        obstacle_on_right = 0
        for i in range(0,4):
            #get x_coor with shift
            x_coor = polygon.points[i].x + 0.2
            #get y_coor with minus sign
            y_coor = -polygon.points[i].y

            if y_coor<0 and y_coor>-0.5 and x_coor <0.5 and x_coor > 0.1:
                obstacle_on_right +=1

        if obstacle_on_right >0:
            self.points_on_lane += obstacle_on_right


        
    

    def check_front(self):
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
        self.points_on_lane =0
        for box_nr in range (len(self.polygons)-1, 0, -1):   
            self.check_polygon_two_lines_middle_four_points(self.polygons[box_nr])
  
        if self.points_on_lane>0:
            self.front_points_counter +=1
            if self.front_points_counter > 5:
                self.front_points_counter = 5
        else:
            self.front_points_counter -=1
        
            if self.front_points_counter<0:
                self.front_points_counter= 0

        if self.front_points_counter>2:
            self.start_maneuver = 1
            self.start_distance = self.distance
            self.front_points_counter = 0
            self.changed_lane = 0

        return self.start_maneuver
    
    def check_end_maneuver(self):
        self.center_dis = self.get_offset(self.lookahead_c,self.c_poly)
        if self.center_dis>0.1:
            self.front_points_counter = 5
            self.start_maneuver= 0
            return 1
        else:
            return 0

    def check_end_maneuver_back(self):
        self.center_dis = self.get_offset(self.lookahead_c,self.c_poly)
        if self.center_dis<-0.1:
            self.front_points_counter = 0
            self.start_maneuver= 0
            return 1
        else:
            return 0

    def check_right_lane(self):
        
        if (len(self.polygons)==0):
            return 0
        self.points_on_lane = 0
        for box_nr in range (len(self.polygons)-1, 0, -1):   
            self.check_polygon_maneuver(self.polygons[box_nr])
        
        if self.points_on_lane>0:
            self.front_points_counter +=1
            if self.front_points_counter > 5:
                self.front_points_counter = 5
        else:
            self.front_points_counter -=1
            if self.front_points_counter<0:
                self.front_points_counter= 0

        if self.front_points_counter==0:
            self.start_maneuver = 1
            self.lane_distance = self.distance - self.start_distance
            self.changed_lane = 0

        return self.start_maneuver
  

    def change_lane_procedure(self):
        self.center_dis = self.get_offset(self.lookahead_c,self.c_poly) 
        if self.center_dis<0:
            self.right_lane = 1
        else:
            self.right_lane = 0


        #normal drive
        if self.trybe ==0:
            start_maneuver = self.check_front()
            if start_maneuver ==1:
                self.trybe = 1
        #going left
        elif self.trybe ==1:
            end_maneuver = self.check_end_maneuver()
            if end_maneuver ==1:
                self.trybe =2
        #wait for no obstacles
        elif self.trybe ==2:
            start_maneuver = self.check_right_lane()
            if start_maneuver ==1:
                self.trybe = 3
        elif self.trybe ==3:
            end_maneuver = self.check_end_maneuver_back()
            if end_maneuver ==1:
                self.trybe =0


        