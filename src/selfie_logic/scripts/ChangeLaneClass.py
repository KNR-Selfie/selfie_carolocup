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
        self.polygon = PolygonStamped()

        #points on current lane
        self.points_on_lane = 0
        self.counter = 0

        self.start_maneuver = 0
        self.start_distance = 0
        self.obstacle_on_right = 0
        self.changed = 0

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
        

    
    def check_polygon_two_lines_four_points(self):
        '''
        Method that checks if points of polygon are inside current lane
        '''
        #check offset of middle lane
        c_dis_c = self.get_offset(self.lookahead_c,self.c_poly)
        points_tmp = 0
        #we are on right so we check right lane
        if c_dis_c<0:
            self.right_lane = 1
            
            for i in range(0,4):
                #get x_coor with shift
                x_coor = self.polygon.polygon.points[i].x + 0.2
                #get y_coor with minus sign
                y_coor = -self.polygon.polygon.points[i].y
                            
                #get y_coor on right and center line with margin
                c_dis_f = self.get_offset(x_coor, self.c_poly)-0.1
                r_dis_f = self.get_offset(x_coor, self.r_poly)+0.1
                               
                if y_coor < r_dis_f and c_dis_f< y_coor and x_coor> 0.3 and x_coor< self.lookahead_max:
                    points_tmp +=1
                    #rospy.loginfo("X: %f C:%f Y: %f R: %f",x_coor,c_dis_f, y_coor,r_dis_f)               
        
        else:
            self.right_lane = 0                      
        
        #return box only if more points are inside
        if points_tmp >1:
            self.points_on_lane += points_tmp

    def check_polygon_two_lines_middle_four_points(self):
        '''
        Method that checks if middle points of polygon are inside current lane
        '''
        #check offset of middle lane
        c_dis_c = self.get_offset(self.lookahead_c,self.c_poly)
        points_tmp = 0
        #we are on right so we check right lane
        if c_dis_c<0:
            self.right_lane = 1
            x_middle =0
            y_middle = 0
            for i in range(0,4):
                #get x_coor with shift
                x_coor = self.polygon.polygon.points[i].x + 0.2
                #get y_coor with minus sign
                y_coor = -self.polygon.polygon.points[i].y

                x_middle +=x_coor
                y_middle +=y_coor
                #get x_coor with shift
                x_coor = self.polygon.polygon.points[i].x + 0.2
                #get y_coor with minus sign
                y_coor = -self.polygon.polygon.points[i].y

            x_middle /=4
            y_middle /=4
            x_min = min(self.polygon.polygon.points[0].x + 0.2,self.polygon.polygon.points[1].x + 0.2,self.polygon.polygon.points[2].x + 0.2,self.polygon.polygon.points[3].x + 0.2)
            y_min = min(-self.polygon.polygon.points[0].y,-self.polygon.polygon.points[1].y,-self.polygon.polygon.points[2].y,-self.polygon.polygon.points[3].y)
            y_max = max(-self.polygon.polygon.points[0].y,-self.polygon.polygon.points[1].y,-self.polygon.polygon.points[2].y,-self.polygon.polygon.points[3].y)
            y_wid = (y_max-y_min)/2

            #get y_coor on right and center line with margin
            c_dis_m = self.get_offset(x_middle, self.c_poly)
            r_dis_m = self.get_offset(x_middle, self.r_poly)
                            
            if c_dis_m<y_middle and y_middle<r_dis_m and x_min>0.3 and x_min<0.7:
                points_tmp +=1      
        
        else:
            self.right_lane = 0                  
        
        #return box only if more points are inside
        if points_tmp >0:
            self.points_on_lane += points_tmp

    def check_polygon_maneuver(self):
        #check offset of middle lane
        c_dis_c = self.get_offset(self.lookahead_c,self.c_poly)
        #we are on right so we check right lane
        if c_dis_c<0:
            self.right_lane = 1
        else:
            self.right_lane = 0

        for i in range(0,4):
            #get x_coor with shift
            x_coor = self.polygon.polygon.points[i].x + 0.2
            #get y_coor with minus sign
            y_coor = -self.polygon.polygon.points[i].y

            if y_coor<0 and y_coor>-0.5 and x_coor <0.5 and x_coor > 0.1:
                self.obstacle_on_right +=1

        
    def get_area(self):
        X = [self.polygon.polygon.points[0].x,self.polygon.polygon.points[1].x,self.polygon.polygon.points[2].x,self.polygon.polygon.points[3].x]
        Y = [self.polygon.polygon.points[0].y,self.polygon.polygon.points[1].y,self.polygon.polygon.points[2].y,self.polygon.polygon.points[3].y]
        area = 0
        j = 4 - 1
        for i in range(0,3):
            area += (X[j]+X[i])*(Y[j]-Y[i])
            j = i
        return (abs(area/2.0))