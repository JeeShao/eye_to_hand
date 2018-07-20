#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import math
# a = np.array([[1,2],[2,1]])
# b = np.array([[0,1],[2,0]])
# print(a.dot(b.T))
CAM = np.mat(' 57 , 53.4 , -40.6, -46.2 ;'
             ' 187.8, 91.4, 193.8, 96.6;'
             ' 647.2, 651.4, 647.5, 623.2;'
             '  1 ,  1 ,  1 ,  1  ')
PIECE = np.mat('0, 100,  0 ,  100 ;'
               '0,  0 , 100,  100 ;'
               '0,  0 ,  0 , 35;'
               '1,  1 ,  1 ,  1' )
CAM_ ,PIECE_ = np.array(CAM.I) ,np.array(PIECE.I)  #CAM_ * X = PIECE_

A = np.linalg.solve(CAM_,PIECE_)
A = np.mat(A)

# A = np.mat('-0.05 , -0.994, -0.03 , 44  ;'
#            '-1.02 ,  0.04 ,   0   , 196 ;'
#            ' 0.1  , -0.02 , -0.52 , 540 ;'
#            '  0   ,   0   ,   0   ,  1  ')

B = A.I
A = np.array(A)
B = np.array(B)
sumA = 0
sumB = 0
for i in A:
    sumA = sumA + math.pow(i[3],2)

for j in B:
    sumB = sumB + math.pow(j[3],2)

# for i in range(A.shape[0]):
#     for j in range(A.shape[1]):
#         A[i][j] = '%.4f' % A[i][j]

print "A:"
print np.mat(A)
print
print "B:"
print np.mat(B)  #B为相机 => 工件

R = np.mat(A[:3,:3])
R_ = R.I   #求逆
R_t = R.T  #转置
print
print "R_"
print R_
print
print "R_t"
print R_t
print

print("A-t:",int(sumA))
print("B-t:",int(sumB))
print(math.sqrt(int(sumA)))
print(math.sqrt(int(sumB)))

