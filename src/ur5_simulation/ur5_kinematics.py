# -*- coding: utf-8 -*-
"""
Created on Wed May 12 20:48:22 2021

@author: Rafael Queiroz
"""

import pandas as pd
import numpy as np
from math import cos, sin, pi

class UR5Kinematics:
        
    def __init__(self, d1=0.089159, a2=-0.42500, a3=-0.39225, d4=0.1091,
                 d5=0.09465, d6=0.0823):
        self.__d1 = d1
        self.__a2 = a2
        self.__a3 = a3
        self.__d4 = d4
        self.__d5 = d5
        self.__d6 = d6 
        
        self._t1, self._t2, self._t3, self._t4, self._t5, self._t6 = (0,0,0,0,0,0)
        
    @property
    def data(self):        
        self._data = np.array([[self._t1, self.__d1, 0, 0],
                         [self._t2, 0, 0, pi/2],
                         [self._t3, 0, self.__a2, 0],
                         [self._t4, self.__d4, self.__a3, 0],
                         [self._t5, self.__d5, 0, pi/2],
                         [self._t6, self.__d6, 0, -pi/2]])

        return self._data
    
    @property
    def dh_matrix(self):
        self._dh_matrix = pd.DataFrame(self.data, columns=['\u03B8'+'i', 'di',
                                                   'a(i-1)','\u03B1'+'(i-1)'],
                               index = ['i1','i2','i3','i4','i5','i6'])    
        return self._dh_matrix

    @staticmethod
    def gen_transf(i):
        
        '''Calculates the transformation matrix between 
        link i [θi,di,a(i-1),α(i-1)] and its predecessor'''               
        
        return np.array([[cos(i[0]), -sin(i[0]), 0, i[2]],
                      [sin(i[0])*cos(i[3]), cos(i[0])*cos(i[3]), -sin(i[3]), -sin(i[3])*i[1]],
                      [sin(i[0])*sin(i[3]), cos(i[0])*sin(i[3]), cos(i[3]), cos(i[3])*i[1]],
                      [0, 0, 0, 1]])       
 
    def get_forward_kinematics(self, q=[0, 0, 0, 0, 0, 0], a=0, b=6, short=True):
        
        '''Receives joint angles vector q and outputs transformation
        matrix from link a to b'''
        
        self._t1 = q[0]
        self._t2 = q[1]
        self._t3 = q[2]
        self._t4 = q[3]
        self._t5 = q[4]
        self._t6 = q[5]
        
        tf = np.eye(4,4)
        for link in range(a,b):
            tf = np.dot(tf, UR5Kinematics.gen_transf(self.dh_matrix.iloc[link]))
        
        if short == True:
            tf = tf.round(3) 
        
        return tf 
    