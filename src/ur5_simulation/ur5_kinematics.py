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
        
        self._teta1, self._teta2, self._teta3, self._teta4, self._teta5, self._teta6 = (0,0,0,0,0,0)
        
    @property
    def data(self):        
        self._data = np.array([[self._teta1, self.__d1, 0, 0],
                         [self._teta2, 0, 0, pi/2],
                         [self._teta3, 0, self.__a2, 0],
                         [self._teta4, self.__d4, self.__a3, 0],
                         [self._teta5, self.__d5, 0, pi/2],
                         [self._teta6, self.__d6, 0, -pi/2]])

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
        
        self._teta1 = q[0]
        self._teta2 = q[1]
        self._teta3 = q[2]
        self._teta4 = q[3]
        self._teta5 = q[4]
        self._teta6 = q[5]
        
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

    def _calculate_teta_1(self, tf, shoulder='left'):
        p05 = np.dot(tf, np.array([[0],[0],[-self.__d6],[1]])) # origin 5 in respect to 1
        p05_x = p05[0][0]
        p05_y = p05[1][0]
        p05_xy = (p05_x**2 + p05_y**2)**0.5
        phi_1 = atan2(p05_y,p05_x)
        phi_2 = acos(self.__d4/p05_xy)
        self._teta1 = phi_1 + phi_2 + pi/2
        
        if shoulder == 'right':
            self._teta1 = phi_1 - phi_2 + pi/2

        return self._teta1
        
    def _calculate_teta_5(self, tf, wrist='down'):
        p06_x = tf[0][3]
        p06_y = tf[1][3]
        num = p06_x*sin(self._teta1) - p06_y*cos(self._teta1) - self.__d4
        if abs(num) >= self.__d6 and num > 0:
            self._teta5 = 0        
        elif abs(num) >= self.__d6 and num < 0:
            self._teta5 = pi
        else:
            self._teta5 = acos(num/self.__d6)

        if wrist == 'up':
            self._teta5 *= -1 

        return self._teta5

    def _calculate_teta_6(self, tf):
        if isclose(sin(self._teta5), 0, abs_tol=self.__zero_tolerance): # indetermination
            self._teta6 = 0
            return self._teta6
        t60 = inv(tf) # transformation matrix from 6 to 0
        num_y = -t60[1][0]*sin(self._teta1) + t60[1][1]*cos(self._teta1) # y numerator
        # print(f'num_y ={num_y}')
        num_x = t60[0][0]*sin(self._teta1) - t60[0][1]*cos(self._teta1) # x numerator
        # print(f'num_x ={num_x}')
        if (isclose(num_y, 0, abs_tol=self.__zero_tolerance) and 
            isclose(num_x, 0, abs_tol=self.__zero_tolerance)): # another indetermination
            self._teta6 = 0
            return self._teta6   
        y = num_y/sin(self._teta5)
        # print(f'y ={y}')
        x = num_x/sin(self._teta5)
        # print(f'x ={x}')
        self._teta6 = atan2(y,x)
        # print(f'teta 1 = {self._teta1}')
        # print(f'teta 6 = {self._teta6}')
        return self._teta6   

    def _calculate_teta_3(self, tf, elbow ='up'):
        t01 = self._gen_transf(self.dh_matrix.iloc[0]) # transformation matrix from 0 to 1
        t45 = self._gen_transf(self.dh_matrix.iloc[4]) # transformation matrix from 4 to 5
        t56 = self._gen_transf(self.dh_matrix.iloc[5]) # transformation matrix from 5 to 6

        # t14 = tf @ inv(t01) @ inv(t45) @ inv(t56) error
        # t14 = tf @ inv(t01) @ inv(np.dot(t45,t56)) error
        # t14 = inv(t01) @ tf @ inv(t45) @ inv(t56) error
        # Conclusion: the order of multiplication matters!!

        # t14 = inv(t01) @ tf @ inv(np.dot(t45,t56)) # transformation matrix from 1 to 4
        t14 = np.dot(np.dot(inv(t01),tf),inv(np.dot(t45,t56))) # transformation matrix from 1 to 4 REWRITE
        self._p14_x = t14[0][3]
        self._p14_z = t14[2][3]
        self._p14_xz = (self._p14_x**2 + self._p14_z**2)**0.5
        div = (self._p14_xz**2 - self.__a2**2 - self.__a3**2)/(2*self.__a2*self.__a3)
        if div < -1:
            self._teta3 = pi
        elif div > 1:
            self._teta3 = 0
        else:
            self._teta3 = acos(div)

        if elbow == 'down':
            self._teta3 *= -1 

        return self._teta3

    def _calculate_teta_2(self, tf):
        
        phi_1 = atan2(-self._p14_z, -self._p14_x)
        phi_2 = asin((-self.__a3*sin(self._teta3))/self._p14_xz)
        self._teta2 = phi_1 - phi_2

        return self._teta2

    def _calculate_teta_4(self, tf):
        t34 = self._gen_transf(self.dh_matrix.iloc[3]) # transformation matrix from 3 to 4
        self._teta4 = atan2(t34[1][0], t34[0][0])

        return self._teta4

    def calculate_inverse_kinematics(self, tf, shoulder='left', wrist='down', elbow='up', short=True):
        q1 = self._calculate_teta_1(tf, shoulder=shoulder)
        q5 = self._calculate_teta_5(tf, wrist=wrist)
        q6 = self._calculate_teta_6(tf)
        q3 = self._calculate_teta_3(tf, elbow=elbow)
        q2 = self._calculate_teta_2(tf)
        q4 = self._calculate_teta_4(tf)

        q = [q1, q2, q3, q4, q5, q6] # joint angles
        q = np.array(q)

        if short == True:
            q = np.around(q, 3)           

        return q

# def main():

    # joints_sets to test code:
    # q_list = [[0, -1.57, 0, 0, 0, 1.32],
    #     [1.20, -0.47, 0.73, 0, 0, 1.32],
    #     [0, -1.57, 0, 0, 0, 1.57],
    #     [1.64, -0.45, 0.45, 0, 3.14, 1.57],
    #     [0, -1.57, 0, 0, 0, 1.57],
    #     [1.39, -0.87, 1.41, -0.5, 1.32, 0],
    #     [0, -1.57, 0, 0, 0, 1.57],
    #     [1.39, -0.89, 1.09, -1.78, -1.57, 0],
    #     [0, -1.57, 0, 0, 0, 1.32]]
    # r = UR5Kinematics()
    # for q in q_list:        
    #     tf = r.calculate_forward_kinematics(q)
    #     print(f'\nfor q = {q}:')
    #     print(f'\ntf:\n{tf}')
    #     print(f'\nDH matrix:\n{r._dh_matrix}')
    #     print(f'\nCalculated inverse matrix from DH matrix:\n{r.calculate_inverse_kinematics(tf)}')
    #     print('-='*30)

# if __name__ == '__main__':
#     main()

# r = UR5Kinematics()
# q = [0, 0, 0, 0, 0, 0]
# tf = r.calculate_forward_kinematics()
