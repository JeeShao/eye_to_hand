#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np

a = np.array([[-1,2],[2,3]])
b = np.array([[3,4],[4,5]])
print '\n a:\n',a
print '\n b:\n',b

##转置
print '\n a transpose:\n',a.T

##共扼矩阵
#print '\n a H:\n',a.I

##逆矩阵
print '\n a inv:\n',np.linalg.inv(a) # 求逆

##转置
print '\n a transpose:\n',a.T

# a + b，矩阵相加
print "\n a+b: \n",a+b

# a - b，矩阵相减
print "\n a-b: \n",a-b

#2x2 矩阵，矩阵相乘
print "\n a mul b:\n",a.dot(b.T)

#2x3矩阵，矩阵点乘
print "\n a dot b: \n",a*b

#2x3矩阵，矩阵点除
print "\n a/b \n:",a/np.linalg.inv(b)

#求迹
print "\n a trace",np.trace(a)

#特征，特征向量
eigval,eigvec = np.linalg.eig(a)
#eigval = np.linalg.eigvals(a) #直接求解特征值

print "\n a eig value:\n",eigval
print'\n a eig vector:\n',eigvec