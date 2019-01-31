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

        self.lookahead_c = 0.0
        self.lookahead_f = 0.3
        self.lookahead_max = 0.5
        #width of road
        self.width = 0.0

        #obstacles polygon
        self.polygon = PolygonStamped()

        #points on current lane
        self.points_on_lane = 0

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

    def check_polygon(self):
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
                #check if x_coor is close enough
                if x_coor> 0.3 and x_coor<self.lookahead_max:
                    #get y_coor 
                    y_coor = -self.polygon.polygon.points[i].y
                    #get y_coor on right and center line with margin
                    c_dis_f = self.get_offset(x_coor, self.c_poly)-0.1
                    r_dis_f = self.get_offset(x_coor, self.r_poly)+0.1
                    #checkout width 
                    self.width = -c_dis_f + r_dis_f
                    #if y_coor is beetween line add point
                    if c_dis_f<y_coor and y_coor<r_dis_f:
                        points_tmp +=1
                
                    
        else:   
            self.right_lane = 0   
            for i in range(0,4):
                x_coor = self.polygon.polygon.points[i].x + 0.2
                if x_coor> 0.3 and x_coor<self.lookahead_max:
                    y_coor = -self.polygon.polygon.points[i].y
                    c_dis_f = self.get_offset(x_coor, self.c_poly)+0.1
                    l_dis_f = self.get_offset(x_coor, self.l_poly)-0.1
                    self.width = +c_dis_f + -l_dis_f
                    if c_dis_f>y_coor and y_coor>l_dis_f:
                        points_tmp +=1

                    
        #return box only if more points are inside
        #if points_tmp >1:
        self.points_on_lane = points_tmp

        

