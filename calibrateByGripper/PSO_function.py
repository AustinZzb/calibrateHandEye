'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 15:55:24
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-04 17:04:38
FilePath: \calibrateHandEye\calibrateByGripper\pso_function_1.py
Description: 因为误差存在的原因, 不能联立等式方程组对球心进行求解，需要使用最优化方程
             AND 求解手眼矩阵的最优化方程
'''
import os
import sys
sys.path.append('./')
from math import sqrt
import numpy as np
import calibrateUtil_scipy as util
import calibrateUtil as myUtil
import math


# 将最优化过程中需要计算的部分抽离出来
class PSO_Util(object):
    # 两点之间的距离
    def getDistance(A, B):
        return sqrt(pow(A['x']-B['x'], 2) + pow(A['y']-B['y'], 2) + pow(A['z']-B['z'], 2))
    
    # 通过OCD计算法兰平面, planes_N
    def plane_equation_from_points(O, params):
        planes_N = []

        for param in params:
            # 从输入点中提取坐标
            x1, y1, z1 = O
            x2, y2, z2 = param[0]
            x3, y3, z3 = param[1]

            # 构建系数矩阵和常数矩阵
            coefficients_matrix = np.array([[x1, y1, z1, 1],
                                            [x2, y2, z2, 1],
                                            [x3, y3, z3, 1]])

            constants_matrix = np.array([[0],
                                        [0],
                                        [0]])

            # 解线性方程组
            solution = np.linalg.solve(coefficients_matrix, constants_matrix)

            # 提取解的值，即平面方程的系数
            plane = solution.flatten()

            planes_N.append(plane)

        # 返回平面方程的系数
        return planes_N
    
    # 点到平面距离
    def distance_to_plane(point, plane_coefficients):
        # 提取平面方程的系数
        A, B, C, D = plane_coefficients

        # 提取点的坐标
        x0, y0, z0 = point

        # 计算点到平面的距离
        distance = abs(A*x0 + B*y0 + C*z0 + D) / math.sqrt(A**2 + B**2 + C**2)

        return distance

    # 计算两个向量夹角的余弦值 - 返回绝对值
    def cosine_similarity(vector1, vector2):
        # 计算点积和向量模
        dot_product = np.dot(vector1, vector2)
        norm_A = np.linalg.norm(vector1)
        norm_B = np.linalg.norm(vector2)

        # 计算余弦值
        cosine_similarity = dot_product / (norm_A * norm_B)

        return abs(cosine_similarity)

    # 通过点坐标字典计算向量
    def get_vector(A, B):
        return np.array([A['x']-B['x'], A['y']-B['y'], A['z']-B['z']])

'''
最优化方程:
求解相机坐标系下的球心坐标
'''
class PSO_function_1():
    def __init__(self):
        self.getDistance = PSO_Util.getDistance

    def setData(self, data):
        self.C, self.D, self.F, self.R = data   

    def pso_function(self, particle):
        # print(len(particle))
        '''使用无约束最优化求解方程组 -> 球心

        Args:
            particle (_type_): _description_

        Returns:
            _type_: _description_
        '''
        # particle: [A['y'], A['z'], E['y'], E['z']]
        # 参见文档方法尝试 z=F['z']
        # (z’-z)**2 - DE**2        = -DF**2
        # (z’-z)**2         + y**2 = AF**2
        #             DE**2 + y**2 = R**2
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}

        EF = self.getDistance(E, self.F)
        DE = self.getDistance(E, self.D)
        AE = self.getDistance(E, A)
        # AE = particle[6]
        AF = self.getDistance(A, self.F)
        DF = self.getDistance(self.D, self.F)

        # 三个勾股定理
        eq1 = EF**2 + DF**2 - DE**2
        eq2 = EF**2 + AE**2 - AF**2
        eq3 = DE**2 + AE**2 - self.R**2

        return abs(eq1) + abs(eq2) + abs(eq3) 
        # mean = (eq1+eq2+eq3)/3
        # return sqrt(pow(eq1-mean, 2) + pow(eq2-mean, 2) + pow(eq3-mean, 2))

    def getConstraintList(self):
        return [self.constraint_ueq_1, self.constraint_ueq_2, self.constraint_ueq_3, self.constraint_ueq_4,
                self.constraint_ueq_5, self.constraint_ueq_6, self.constraint_ueq_7]

    # constraint_ueq_1约束AD=R, AC=R
    def constraint_ueq_1(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return self.getDistance(A, self.D) -self.R

    def constraint_ueq_2(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return self.getDistance(A, self.C) -self.R

    # 约束C, D 处于以点A为球心的球上
    def constraint_ueq_3(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return (self.D['x'] - A['x'])**2 + (self.D['y'] - A['y'])**2 + (self.D['z'] - A['z'])**2 - self.R**2
    
    def constraint_ueq_4(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return (self.C['x'] - A['x'])**2 + (self.C['y'] - A['y'])**2 + (self.C['z'] - A['z'])**2 - self.R**2
    
    # DE < AD
    def constraint_ueq_5(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}

        return self.getDistance(self.D, E) - self.getDistance(self.D, A)
    
    # AE < R
    def constraint_ueq_6(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}
        
        return self.getDistance(A, E) - self.R
    
    def constraint_ueq_7(self, particle):
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}
        
        return self.getDistance(self.D, E) - self.getDistance(self.C, E)


# 最优化函数求解手眼矩阵
class PSO_function_2(object):
    def __init__(self):
        self.getDistance = PSO_Util.getDistance
        self.plane_equation_from_points = PSO_Util.plane_equation_from_points
        self.distance_to_plane = PSO_Util.distance_to_plane
        self.cosine_similarity = PSO_Util.cosine_similarity
        self.get_vector = PSO_Util.get_vector
    
    # 初始化数据
    def setData(self, pose, params, sphericalCenters):
        self.base2gripperList = pose
        self.params = params
        self.sphericalCenters = sphericalCenters

    # 法兰平面是唯一的, 所以由弦端点和法兰中心确定的所有平面应该是重合的
    # 拆分以下就是, 所有平面应该是平行的, 距离为0的
    def pso_function(self, particle):
        # 将粒子(手眼矩阵)转换为齐次变换矩阵
        matrix = util.pose2Homo([particle[:6]])[0]
        gripperCenter = particle[6:]

        #   E<-B<-C<-O
        # 将相机坐标系下的点转换到法兰坐标系下
        for param, sphericalCenter in zip(self.params, self.sphericalCenters):

            sphericalCenter = np.array(sphericalCenter.append(1)).reshape(4, 1)

        pointsOnGripper = []
        for base2gripper in self.base2gripperList:
            point = base2gripper @ matrix @ sphericalCenter

            pointsOnGripper.append(point)

        meanPoint = np.mean(pointsOnGripper, axis=0)
        return np.sum(np.square(pointsOnGripper - meanPoint))
