# -*- coding: utf-8 -*-
"""
Created on Wed May 12 20:48:22 2021

@author: Rafael Queiroz
"""

import pandas as pd
import numpy as np
from math import cos, sin

class UR5:
        
    def __init__(self, d1=0.089159, a2=-0.42500, a3=-0.39225, d4=0.1091,
                 d5=0.09465, d6=0.0823):
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6 
        self.t1, self.t2, self.t3, self.t4, self.t5, self.t6 = (0,0,0,0,0,0)
           
    @property
    def data(self):
        return np.array([[self.t1, self.d1, 0, 0],
                         [self.t2, 0, 0, 90],
                         [self.t3, 0, self.a2, 0],
                         [self.t4, self.d4, self.a3, 0],
                         [self.t5, self.d5, 0, 90],
                         [self.t6, self.d6, 0, -90]])
    
    
    @property
    def dh(self):        
        return pd.DataFrame(self.data, columns=['\u03B8'+'i', 'di',
                                                   'a(i-1)','\u03B1'+'(i-1)'],
                               index = ['i1','i2','i3','i4','i5','i6'])    

    @staticmethod
    def gen_transf(i):
        
        '''Calculates the transformation matrix between 
        link i [θi,di,a(i-1),α(i-1)] and its predecessor'''
               
        
        t = np.array([[cos(i[0]), -sin(i[0]), 0, i[2]],
                      [sin(i[0])*cos(i[3]), cos(i[0])*cos(i[3]), -sin(i[3]), -sin(i[3])*i[1]],
                      [sin(i[0])*sin(i[3]), cos(i[0])*sin(i[3]), cos(i[3]), cos(i[3])*i[1]],
                      [0, 0, 0, 1]])
 
        
        return t    
    
    @property
    def transf_matrix(self, a=0, b=6):
        
        '''Calculates the transformation matrix between links a and b'''
        
        tf = np.eye(4,4)
        for link in range(a,b):
            tf = np.dot(tf, UR5.gen_transf(self.dh.iloc[link]))
            
        return tf  
               
    
    def fkine(self, q=[0, 0, 0, 0, 0, 0]):
        
        '''Receives joint angles vector q and outputs transformation
        matrix from T0 to T6'''
        
        self.t1 = q[0]
        self.t2 = q[1]
        self.t3 = q[2]
        self.t4 = q[3]
        self.t5 = q[4]
        self.t6 = q[5]
        
        return self.transf_matrix
    

def main():
        
    r = UR5() 
    q0 = [0, 0, 0, 0, 0, 0]    
    q1 = [0, -1.57, 0, 0, 0, 1.32]    
    q2 = [1.20, -0.47, 0.73, 0, 0, 1.32] 
    print(f'\nTransformation matrix for q = {q0}:\n\n{r.fkine(q0)}')
    print(f'\nCorrespondent Denavit-Hartenberg Parameters:\n\n{r.dh}\n')
    print('-='*40)
    print(f'\nTransformation matrix for q = {q1}:\n\n{r.fkine(q1)}')
    print(f'\nCorrespondent Denavit-Hartenberg Parameters:\n\n{r.dh}')
    print('-='*40)
    print(f'\nTransformation matrix for q = {q2}:\n\n{r.fkine(q2)}')
    print(f'\nCorrespondent Denavit-Hartenberg Parameters:\n\n{r.dh}')
 
    
if __name__ == '__main__':
    main()
