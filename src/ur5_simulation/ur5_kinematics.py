# -*- coding: utf-8 -*-
"""
Created on Wed May 12 20:48:22 2021

@author: Rafael Queiroz
"""

import pandas as pd 
import numpy as np
from math import acos, asin, atan2, cos, sin, pi
from numpy.linalg import inv

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

class UR5Kinematics:
        
    def __init__(self, d1=0.089159, a2=-0.42500, a3=-0.39225, d4=0.1091,
                 d5=0.09465, d6=0.0823):
        self.__d1 = d1
        self.__a2 = a2
        self.__a3 = a3
        self.__d4 = d4
        self.__d5 = d5
        self.__d6 = d6

        self.__zero_tolerance = 0.01
        
        self._theta1, self._theta2, self._theta3, self._theta4, self._theta5, self._theta6 = (0,0,0,0,0,0)
        
    @property
    def data(self):        
        self._data = np.array([[self._theta1, self.__d1, 0, 0],
                         [self._theta2, 0, 0, pi/2],
                         [self._theta3, 0, self.__a2, 0],
                         [self._theta4, self.__d4, self.__a3, 0],
                         [self._theta5, self.__d5, 0, pi/2],
                         [self._theta6, self.__d6, 0, -pi/2]])

        return self._data
    
    @property
    def dh_matrix(self):
        self._dh_matrix = pd.DataFrame(self.data, columns=['\u03B8'+'i', 'di','a(i-1)','\u03B1'+'(i-1)'], index = ['i1','i2','i3','i4','i5','i6'])    
        return self._dh_matrix

    def _gen_transf(self,i):
        
        '''Calculates the transformation matrix between link i and i+1'''               
        
        return np.array([[cos(i[0]), -sin(i[0]), 0, i[2]],
                      [sin(i[0])*cos(i[3]), cos(i[0])*cos(i[3]), -sin(i[3]), -sin(i[3])*i[1]],
                      [sin(i[0])*sin(i[3]), cos(i[0])*sin(i[3]), cos(i[3]), cos(i[3])*i[1]],
                      [0, 0, 0, 1]])       
 
    ##############  FORWARD KINEMATICS CALCULATION  ############## 

    def calculate_forward_kinematics(self, q=[0, 0, 0, 0, 0, 0], a=0, b=6, short=True):
        
        '''Receives joint angles vector q and outputs transformation
        matrix from link a to b'''
        
        self._theta1 = q[0]
        self._theta2 = q[1]
        self._theta3 = q[2]
        self._theta4 = q[3]
        self._theta5 = q[4]
        self._theta6 = q[5]
        
        tf = np.eye(4,4)
        for link in range(a,b):
            tf = np.dot(tf, self._gen_transf(self.dh_matrix.iloc[link]))
        
        if short == True:
            tf = tf.round(3) 
        
        return tf 

    ##############  INVERSE KINEMATICS CALCULATION  ############## 

    def set_zero_tolerance(self, tolerance):
        if tolerance > 0:
            self.__zero_tolerance = tolerance

    def _calculate_theta_1(self, tf, shoulder='left'):
        p05 = np.dot(tf, np.array([[0],[0],[-self.__d6],[1]])) # origin 5 in respect to 1
        p05_x = p05[0][0]
        p05_y = p05[1][0]
        p05_xy = (p05_x**2 + p05_y**2)**0.5
        phi_1 = atan2(p05_y,p05_x)
        phi_2 = acos(self.__d4/p05_xy)
        self._theta1 = phi_1 + phi_2 + pi/2
        
        if shoulder == 'right':
            self._theta1 = phi_1 - phi_2 + pi/2

        return self._theta1
        
    def _calculate_theta_5(self, tf, wrist='down'):
        p06_x = tf[0][3]
        p06_y = tf[1][3]
        num = p06_x*sin(self._theta1) - p06_y*cos(self._theta1) - self.__d4
        if abs(num) >= self.__d6 and num > 0:
            self._theta5 = 0        
        elif abs(num) >= self.__d6 and num < 0:
            self._theta5 = pi
        else:
            self._theta5 = acos(num/self.__d6)

        if wrist == 'up':
            self._theta5 *= -1 

        return self._theta5

    def _calculate_theta_6(self, tf):
        if isclose(sin(self._theta5), 0, abs_tol=self.__zero_tolerance): # indetermination
            self._theta6 = 0
            return self._theta6
        t60 = inv(tf) # transformation matrix from 6 to 0
        num_y = -t60[1][0]*sin(self._theta1) + t60[1][1]*cos(self._theta1) # y numerator
        num_x = t60[0][0]*sin(self._theta1) - t60[0][1]*cos(self._theta1) # x numerator
        if (isclose(num_y, 0, abs_tol=self.__zero_tolerance) and 
            isclose(num_x, 0, abs_tol=self.__zero_tolerance)): # another indetermination
            self._theta6 = 0
            return self._theta6   
        y = num_y/sin(self._theta5)
        x = num_x/sin(self._theta5)
        self._theta6 = atan2(y,x)

        return self._theta6   

    def _calculate_theta_3(self, tf, elbow ='up'):
        t01 = self._gen_transf(self.dh_matrix.iloc[0]) # transformation matrix from 0 to 1
        t45 = self._gen_transf(self.dh_matrix.iloc[4]) # transformation matrix from 4 to 5
        t56 = self._gen_transf(self.dh_matrix.iloc[5]) # transformation matrix from 5 to 6

        # t14 = tf @ inv(t01) @ inv(t45) @ inv(t56) error
        # t14 = tf @ inv(t01) @ inv(np.dot(t45,t56)) error
        # t14 = inv(t01) @ tf @ inv(t45) @ inv(t56) error
        # Conclusion: the order of multiplication matters!!

        # t14 = inv(t01) @ tf @ inv(np.dot(t45,t56)) # transformation matrix from 1 to 4 (python 3)
        t14 = np.dot(np.dot(inv(t01),tf),inv(np.dot(t45,t56))) # transformation matrix from 1 to 4 (python 2 & 3)
        self._p14_x = t14[0][3]
        self._p14_z = t14[2][3]
        self._p14_xz = (self._p14_x**2 + self._p14_z**2)**0.5
        div = (self._p14_xz**2 - self.__a2**2 - self.__a3**2)/(2*self.__a2*self.__a3)
        if div < -1:
            self._theta3 = pi
        elif div > 1:
            self._theta3 = 0
        else:
            self._theta3 = acos(div)

        if elbow == 'down':
            self._theta3 *= -1 

        return self._theta3

    def _calculate_theta_2(self, tf):
        
        phi_1 = atan2(-self._p14_z, -self._p14_x)
        phi_2 = asin((-self.__a3*sin(self._theta3))/self._p14_xz)
        self._theta2 = phi_1 - phi_2

        return self._theta2

    def _calculate_theta_4(self, tf):
        t01 = self._gen_transf(self.dh_matrix.iloc[0]) # transformation matrix from 0 to 1
        t12 = self._gen_transf(self.dh_matrix.iloc[1]) # transformation matrix from 1 to 2
        t23 = self._gen_transf(self.dh_matrix.iloc[2]) # transformation matrix from 2 to 3
        t45 = self._gen_transf(self.dh_matrix.iloc[4]) # transformation matrix from 4 to 5
        t56 = self._gen_transf(self.dh_matrix.iloc[5]) # transformation matrix from 5 to 6
        t30 = inv(np.dot(np.dot(t01,t12),t23)) # transformation matrix from 3 to 0
        t64 = inv(np.dot(t45,t56)) # transformation matrix from 6 to 4
        t34 = np.dot(np.dot(t30,tf),t64) # transformation matrix from 3 to 4
        self._theta4 = atan2(t34[1][0], t34[0][0])

        return self._theta4

    def calculate_inverse_kinematics(self, tf, shoulder='left', wrist='up', elbow='up', short=True):
        q1 = self._calculate_theta_1(tf, shoulder=shoulder)
        q5 = self._calculate_theta_5(tf, wrist=wrist)
        q6 = self._calculate_theta_6(tf)
        q3 = self._calculate_theta_3(tf, elbow=elbow)
        q2 = self._calculate_theta_2(tf)
        q4 = self._calculate_theta_4(tf)

        q = [q1, q2, q3, q4, q5, q6] # joint angles
        q = np.array(q)

        if short == True:
            q = np.around(q, 3)           

        return q

# joints_sets to test code:                   # best configuration of wrist 
#   - [0.00, -1.57, 0.00, 0.00, 0.00, 0.00]   # 0 up/down
#   - [0.95, -0.47, 0.73, 0.00, -0.31, 1.32]  # 1 up
#   - [1.09, -0.47, 0.73, 0.00, -0.20, 1.32]  # 2 up
#   - [0.95, -0.47, 0.73, 0.00, -0.31, 1.32]  # 3 up
#   - [0.00, -1.57, 0.00, 0.00, 0.00, 0.00]   # 4 up/down
#   - [1.94, -0.45, 0.00, 0.00, 3.27, 1.57]   # 5 up
#   - [1.94, -0.45, 0.45, 0.00, 3.27, 1.57]   # 6 up
#   - [1.76, -0.45, 0.45, 0.00, 3.22, 1.57]   # 7 up
#   - [1.94, -0.45, 0.45, 0.00, 3.27, 1.57]   # 8 up
#   - [1.94, -0.45, 0.00, 0.00, 3.27, 1.57]   # 9 up
#   - [0.00, -1.57, 0.00, 0.00, 0.00, 0.00]   # 10 up/down
#   - [1.00, -1.25, 2.07, -0.75, 1.33, 1.57]  # 11 down
#   - [1.33, -1.25, 2.07, -0.75, 1.33, 1.57]  # 12 down
#   - [1.33, -1.20, 1.88, -0.60, 1.31, 1.57]  # 13 down
#   - [1.33, -1.25, 2.07, -0.75, 1.33, 1.57]  # 14 down
#   - [1.00, -1.25, 2.07, -0.75, 1.33, 1.57]  # 15 down
#   - [0.00, -1.57, 0.00, 0.00, 0.00, 0.00]   # 16 up/down
#   - [1.44, -0.96, 0.75, -1.49, -1.57, 0.00] # 17 up
#   - [1.44, -0.84, 0.75, -1.46, -1.57, 0.00] # 18 up
#   - [1.44, -0.96, 0.75, -1.49, -1.57, 0.00] # 19 up
#   - [0.00, -1.57, 0.00, 0.00, 0.00, 0.00]   # 20 up/down
