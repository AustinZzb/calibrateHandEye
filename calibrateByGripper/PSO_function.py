'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 15:55:24
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-12-22 15:59:05
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

        self.C, self.D, self.R, self.F, self.AF, self.DF = dataLoader.getData()

    def pso_function(self, particle):
        '''使用无约束最优化求解方程组 -> 球心

        Args:
            particle (_type_): _description_

        Returns:
            _type_: _description_
        '''
        # particle: [y, z', DE(r)]
        # 参见文档方法尝试 z=F['z']
        # (z’-z)**2 - DE**2        = -DF**2
        # (z’-z)**2         + y**2 = AF**2
        #             DE**2 + y**2 = R**2
        A = {'x': self.F['x'], 'y': particle[0], 'z': particle[1]}

        EF = abs(particle[1]-self.F['z'])
        DE = abs(particle[2])
        AE = abs(particle[0])

        # 三个勾股定理
        eq1 = EF**2 + self.DF**2 - DE**2
        eq2 = EF**2 + AE**2 - self.AF**2
        eq3 = DE**2 + AE**2 - self.R**2

        # print('eq1  : %f = (%f - %f) + %f - %f' % (eq1, particle[1], F['z'], DF, particle[2]))
        # print('eq2  : %f = (%f - %f) + %f - %f' % (eq2, particle[1], F['z'], particle[0], AF))
        # print('eq3  : %f = %f + %f - %f' % (eq3, particle[2], particle[0], R))
        # print('=' * 20)

        # 三棱锥体积, 分别以DEF为底和AEF为底
        V = self.DF*EF*AE - AE*EF*self.DF

        # 将弦两个端点约束在与法兰同半径的球上
        L = (self.D['x'] - A['x'])**2 + (self.D['y'] - A['y'])**2 + (self.D['z'] - A['z'])**2 - self.R**2 + \
            (self.C['x'] - A['x'])**2 + (self.C['y'] - A['y'])**2 + (self.C['z'] - A['z'])**2 - self.R**2

        return abs(eq1) + abs(eq2) + abs(eq3) + V + L
        # mean = (eq1+eq2+eq3)/3
        # return sqrt(pow(eq1-mean, 2) + pow(eq2-mean, 2) + pow(eq3-mean, 2))

    def getConstraintList(self):
        return [self.constraint_ueq_1, self.constraint_ueq_2, self.constraint_ueq_2]

    def constraint_ueq_1(self, particle):
        return particle[2] - self.R

    def constraint_ueq_2(self, particle):
        return abs(particle[1] - self.F['z']) - self.R

    def constraint_ueq_3(self, particle):
        return abs(particle[0]) - self.R


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
