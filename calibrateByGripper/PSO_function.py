'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 15:55:24
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-10 17:08:22
FilePath: \calibrateHandEye\calibrateByGripper\pso_function_1.py
Description: 因为误差存在的原因, 不能联立等式方程组对球心进行求解，需要使用最优化方程
             AND 求解手眼矩阵的最优化方程
'''
import re
import inspect
import math
import calibrateUtil_scipy as util
import numpy as np
from math import sqrt
import sys
from functools import partial
sys.path.append('./')

# 将最优化过程中需要计算的部分抽离出来


class PSO_Util(object):
    # 两点之间的距离
    def getDistanceByDict(A, B):
        return sqrt(pow(A['x']-B['x'], 2) + pow(A['y']-B['y'], 2) + pow(A['z']-B['z'], 2))

    # 通过OCD计算法兰平面, planes_N
    def plane_equation_from_points(O, params):
        '''计算OCD平main

        Args:
            O (ndarray): 法兰中心particle[6:]
            params (ndarray): 法兰坐标系下的CDF

        Returns:
            ndarray: 返回计算得到的平面[:3]即为法向量
        '''
        # 计算向量v1和v2
        planes_N = []

        p1 = O
        for param in params:
            # 从输入点中提取坐标
            p2 = param[0]
            p3 = param[1]

            v1 = p2 - p1
            v2 = p3 - p1

            # 计算向量n
            n = np.cross(v1, v2)

            # 计算单位向量u
            u = n / np.linalg.norm(n)

            # 计算平面方程系数
            A, B, C = u
            D = -(A * p1[0] + B * p1[1] + C * p1[2])

            planes_N.append([A, B, C, D])

        # 返回平面方程系数
        return planes_N

    # 点到平面距离
    def distance_to_plane(point, plane_coefficients, infoFlag=False):
        # 提取平面方程的系数
        A, B, C, D = plane_coefficients

        # 提取点的坐标
        x0, y0, z0 = point

        # 计算点到平面的距离
        distance = abs(A*x0 + B*y0 + C*z0 + D) / math.sqrt(A**2 + B**2 + C**2)

        if infoFlag:
            print(f'平面方程：{A}x+{B}y+{C}z+{D}, 点坐标{x0, y0, z0}')
            print(f'点到平面距离{distance}')

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

    # 将相机坐标系下的坐标转换至基坐标系下
    def camera2gripper_points(params_, sphericalCenters, base2gripperList, particle):
        '''
            @params:
                params: C、D、R
                sphericalCenters: 球心坐标
                base2gripperList: 机器人位姿
                particle: 最优化求解参数

            @return:
                paramsOnGripper[poseLen][3][3]
                centerOnGripper[poseLen][N][3]
        '''

        # 将粒子(手眼矩阵)转换为齐次变换矩阵
        matrix = util.pose2Homo([particle[:6]])[0]

        #   E<-B<-C<-O
        # 将相机坐标系下的点转换到法兰坐标系下
        paramsOnGripper = []
        centersOnGripper = []

        poseLen = len(params_)
        for i in range(poseLen):
            base2gripper = base2gripperList[i]

            temp_paramsOnGripper = []
            temp_centersOnGripper = []

            for j in range(3):
                # 将CD坐标字典转换为np数组
                point = params_[i][j]
                temp = list(point.values())
                temp.append(1)
                point = np.array(temp).reshape(4, 1)
                pointOnGripper = base2gripper @ matrix @ point

                temp_paramsOnGripper.append(
                    pointOnGripper.reshape(1, 4)[0][:3])

            for j in range(len(sphericalCenters[i])):
                # 将球心坐标转换为(4, 1)向量
                point = sphericalCenters[i][j][0]
                # print(point)
                point = np.array(np.append(point, 1)).reshape(4, 1)

                centerOnGripper = base2gripper @ matrix @ point

                temp_centersOnGripper.append(
                    centerOnGripper.reshape(1, 4)[0][:3])

            paramsOnGripper.append(temp_paramsOnGripper)
            centersOnGripper.append(temp_centersOnGripper)

        return paramsOnGripper, centersOnGripper
        # return paramsOnGripper

    # 将相机坐标系下的某个坐标转换至基坐标系下
    def camera2gripper_point(point, base2gripper, matirxParticle):
        '''
            @params:
                params: C、D、R
                sphericalCenters: 球心坐标
                base2gripperList: 机器人位姿
                particle: 最优化求解参数

            @return:
                paramsOnGripper[poseLen][3][3]
                centerOnGripper[poseLen][N][3]
        '''

        # 将粒子(手眼矩阵)转换为齐次变换矩阵
        matrix = util.pose2Homo([matirxParticle])[0]
        # gripperCenter = particle[6:]

        #   E<-B<-C<-O
        # 将相机坐标系下的点转换到法兰坐标系下
        # 将CD坐标字典转换为np数组
        temp = list(point.values())
        temp.append(1)
        point = np.array(temp).reshape(4, 1)
        pointOnGripper = base2gripper @ matrix @ point

        return pointOnGripper.reshape(1, 4)[0][:3]

# 最优化方程: 求解相机坐标系下的球心坐标


class PSO_function_1():
    def __init__(self):
        self.getDistance = PSO_Util.getDistanceByDict

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
        return self.getDistance(A, self.D) - self.R

    def constraint_ueq_2(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return self.getDistance(A, self.C) - self.R

    # 约束C, D 处于以点A为球心的球上
    def constraint_ueq_3(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return (self.D['x'] - A['x'])**2 + (self.D['y'] - A['y'])**2 + (self.D['z'] - A['z'])**2 - self.R**2

    def constraint_ueq_4(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        return (self.C['x'] - A['x'])**2 + (self.C['y'] - A['y'])**2 + (self.C['z'] - A['z'])**2 - self.R**2

    # DE < R
    def constraint_ueq_5(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}

        return self.getDistance(self.D, E) - self.getDistance(self.D, A)

    # AE < R
    def constraint_ueq_6(self, particle):
        A = {'x': particle[0], 'y': particle[1], 'z': particle[2]}
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}

        return self.getDistance(A, E) - self.R

    # DE = CE
    def constraint_ueq_7(self, particle):
        E = {'x': particle[3], 'y': particle[4], 'z': particle[5]}

        return self.getDistance(self.D, E) - self.getDistance(self.C, E)


# 最优化函数求解手眼矩阵
class PSO_function_2(object):
    def __init__(self):
        # 工具函数
        self.Util = PSO_Util.getDistanceByDict
        self.plane_equation_from_points = PSO_Util.plane_equation_from_points
        self.distance_to_plane = PSO_Util.distance_to_plane
        self.cosine_similarity = PSO_Util.cosine_similarity
        self.get_vector = PSO_Util.get_vector
        self.camera2gripper_points = PSO_Util.camera2gripper_points
        self.camera2gripper_point = PSO_Util.camera2gripper_point

    # 初始化[base2gripperList, params, sphericalCenters]
    def setData(self, data):
        self.base2gripperList, self.params, self.sphericalCenters, self.plane_M = data
        self.R = self.params[0][3]

    # 初始化数据
    def initData(self, particle):
        # 求解
        # self.paramsOnGripper, self.centersOnGripper = self.camera2gripper_points(
        #     self.params, self.sphericalCenters, self.base2gripperList, particle)

        self.paramsOnGripper, _ = self.camera2gripper_points(
            self.params, self.sphericalCenters, self.base2gripperList, particle)

        self.plane_N = self.plane_equation_from_points(
            particle[6:], self.paramsOnGripper)

    # 法兰平面是唯一的, 所以由弦端点和法兰中心确定的所有平面应该是重合的
    # 拆分以下就是, 所有平面应该是平行的, 距离为0的
    def pso_function(self, particle):
        self.initData(particle)

        distance_sum = 0
        poseLen = len(self.params)

        # 约束第i组C、D到第(i+1)%poseLen个平面的距离为0, 即所有CD共面，即所有N为一个平面
        for i in range(poseLen):
            pointIndex = i
            planeIndex = (i+1) % poseLen

            distance_sum += self.distance_to_plane(
                self.paramsOnGripper[pointIndex][0], self.plane_N[planeIndex])
            distance_sum += self.distance_to_plane(
                self.paramsOnGripper[pointIndex][1], self.plane_N[planeIndex])

        return distance_sum

    def getConstraintList(self):
        func = []
        poseLen = len(self.plane_M)

        # 循环创建lambda表达式并使用循环变量的话应该将循环变量作为参数传入
        for i in range(poseLen):
            for k in range(poseLen):
                if i == k:
                    continue
                # 约束平面Ni平行，即法向量平行
                func.append(partial(self.constraint_plane_N, plane_index=i, other_plane_index=k, infoFlag=False))
                # CiDi到其他任意平面Nk的距离为0
                func.append(partial(self.constraint_CD_N_distance, point_index=i, plane_index=k, CD_index=0, infoFlag=False))
                func.append(partial(self.constraint_CD_N_distance, point_index=i, plane_index=k, CD_index=1, infoFlag=False))

            for j in range(2):
                # 约束弦端点到点O距离为R
                func.append(partial(self.constraint_O_CD_distance, i=i, j=j, infoFlag=False))
        
        return func

    # def getConstraintList(self):
    #     return [self.constraint_ueq_3, self.constraint_ueq_1, self.constraint_ueq_2_1, self.constraint_ueq_2_2]

    # # 约束弦端点到点O距离为R
    # def constraint_ueq_1(self, particle):
    #     sum = 0

    #     for i in range(len(self.base2gripperList)):
    #         for j in range(2):
    #             sum += self.constraint_O_CD_distance(particle, i, j, infoFlag=False)

    #     return sum
        
    # # CiDi到其他任意平面Nk的距离为0
    # def constraint_ueq_2_1(self, particle):
    #     sum = 0

    #     for i in range(len(self.base2gripperList)):
    #         for j in range(len(self.base2gripperList)):
    #             if i == j:
    #                 continue
    #             sum += self.constraint_CD_N_distance(particle, i, j, 0, infoFlag=True)

    #     return sum
    
    # def constraint_ueq_2_2(self, particle):
    #     sum = 0

    #     for i in range(len(self.base2gripperList)):
    #         for j in range(len(self.base2gripperList)):
    #             if i == j:
    #                 continue
    #             sum += self.constraint_CD_N_distance(particle, i, j, 1, infoFlag=True)

    #     return sum
            
    # # 约束平面Ni平行，即法向量平行
    # def constraint_ueq_3(self, particle):
        sum = 0

        for i in range(len(self.base2gripperList)):
            for j in range(len(self.base2gripperList)):
                if i == j:
                    continue
                sum += self.constraint_plane_N(particle, i, j, infoFlag=False)

        return sum

    def getOnePointInGripper(self, index, matrix):
        pose = self.base2gripperList[index]
        C = self.camera2gripper_point(self.params[index][0], pose, matrix)
        D = self.camera2gripper_point(self.params[index][1], pose, matrix)

        return C, D

    def constraint_O_CD_distance(self, particle, i, j, infoFlag):
        result = self.R - np.linalg.norm(
            self.camera2gripper_point(self.params[i][j], self.base2gripperList[i],
                                      particle[:6]) - particle[6:])
        if infoFlag:
            print(f'C{i}到点O的距离为: ', result) if j == 0 else print(
                f'D{i}到点O的距离为: ', result)
        return result

    def constraint_CD_N_distance(self, particle, point_index, plane_index, CD_index, infoFlag):
        matrix = particle[:6]
        O = particle[6:]
        C_, D_ = self.getOnePointInGripper(plane_index, matrix)

        plane_equation = self.plane_equation_from_points(O, [[C_, D_]])[0]

        C, D = self.getOnePointInGripper(point_index, matrix)
        point = C if CD_index == 0 else D

        result = 0-self.distance_to_plane(point, plane_equation, infoFlag)
        if infoFlag:
            print(f'C{point_index}到平面N{plane_index}的距离: ', result)
        return result

    # 求出plane_index和这个坐标加随机数求得的两个平面的余弦值-1的差
    def constraint_plane_N(self, particle, plane_index, other_plane_index, infoFlag):
        matrix = particle[:6]
        O = particle[6:]

        C_1, D_1 = self.getOnePointInGripper(plane_index, matrix)
        C_2, D_2 = self.getOnePointInGripper(other_plane_index, matrix)

        plane_equation_1 = self.plane_equation_from_points(O, [[C_1, D_1]])[0]
        plane_equation_2 = self.plane_equation_from_points(O, [[C_2, D_2]])[0]

        result = self.cosine_similarity(
            plane_equation_1[:3], plane_equation_2[:3]) - 1

        if infoFlag:
            print(f'平面{plane_index}与平面{other_plane_index}余弦值为: ', result)

        return result

    # 求出法兰中新O到其他平面N的距离
    # def constraint_O_N_distance(self, particle, plane_index, infoFlag):
    #     matrix = particle[:6]
    #     O = particle[6:]

    #     C, D = self.getOnePointInGripper(plane_index, matrix)

    #     plane_equation = self.plane_equation_from_points(O, [[C, D]])[0]

    #     result = self.distance_to_plane(O, plane_equation, infoFlag)

    #     if infoFlag:
    #         print('点O到平面Ni的距离: ', result)
    #     return result

    # # 约束平面Ni和平面Mi垂直，即法向量垂直
    # def constraint_ueq_2(self, particle):
    #     poseLen = len(self.plane_N)
    #     sum = 0

    #     for i in range(poseLen):
    #         temp = self.plane_M[i][:3] @ self.plane_N[(i+3)%poseLen][:3]
    #         print(temp)
    #         sum += temp

    #     return sum

    # 约束平面Ni平行，即法向量平行
    # def constraint_ueq_1(self, particle):
    #     # CDO平面N
    #     p1 = particle[6:]
    #     p2 = param[0]
    #     p3 = param[1]

    #     v1 = p2 - p1
    #     v2 = p3 - p1

    #     # 计算向量n
    #     n = np.cross(v1, v2)

    #     # 计算单位向量u
    #     u = n / np.linalg.norm(n)

    #     plane_N = self.plane_equation_from_points(particle[6:], self.paramsOnGripper)
    #     poseLen = len(plane_N)
    #     sum = 0

    #     for i in range(poseLen):
    #         temp = self.cosine_similarity(plane_N[i][:3], plane_N[(i+2)%poseLen][:3])
    #         sum += temp

    #     return poseLen - sum
