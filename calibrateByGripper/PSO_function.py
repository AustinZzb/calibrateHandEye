'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 15:55:24
LastEditors: Austin 58591300+Justin-zzb@users.noreply.github.com
LastEditTime: 2023-12-22 21:44:01
FilePath: \calibrateHandEye\calibrateByGripper\pso_function_1.py
Description: 因为误差存在的原因, 不能联立等式方程组对球心进行求解，需要使用最优化方程
             AND 求解手眼矩阵的最优化方程
'''
from math import sqrt
import numpy as np
import calibrateUtil_scipy as util
import calibrateUtil as myUtil


class PSO_function_1():
    def setDataLoader(self, dataLoader):
        self.handle = dataLoader

        self.C, self.D, self.R, self.F, self.DF = dataLoader.getData()

    def getDistance(self, A, B):
        return sqrt(pow(A['x']-B['x'], 2) + pow(A['y']-B['y'], 2) + pow(A['z']-B['z'], 2))

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

        # 三个勾股定理
        eq1 = EF**2 + self.DF**2 - DE**2
        eq2 = EF**2 + AE**2 - AF**2
        eq3 = DE**2 + AE**2 - self.R**2

        # 三棱锥体积, 分别以DEF为底和AEF为底
        V = self.DF*EF*AE - AE*EF*self.DF

        # print(abs(eq1) , abs(eq2) , abs(eq3) , abs(V) , abs(L), abs(eq1) + abs(eq2) + abs(eq3) + abs(V) + abs(L))
        return abs(eq1) + abs(eq2) + abs(eq3) + abs(V)
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


# class PSO_function_2(object):

#     def setSphericalCenter(point):
#         sphericalCenter = point

#     path = 'Data\LMI_gripper_calibrate'
#     base2gripperList = DataLoader.getData_2(path)

#     sphericalCenter = [1,2]

#     def pso_function_2(particle):
#         # 将粒子转换为齐次变换矩阵
#         matrix = util.pose2Homo([particle])[0]

#         #   E<-B<-C<-O
#         sphericalCenter = np.array(sphericalCenter.append(1)).reshape(4, 1)

#         pointsOnGripper = []
#         for base2gripper in base2gripperList:
#             point = base2gripper @ matrix @ sphericalCenter

#             pointsOnGripper.append(point)

#         meanPoint = np.mean(pointsOnGripper, axis=0)
#         return np.sum(np.square(pointsOnGripper - meanPoint))
