'''
Author: Austin 1158680540@qq.com
Date: 2023-12-20 15:14:53
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-08 16:47:25
FilePath: \calibrateHandEye\calibrateByGripper\run.py
Description: 球心 & 手眼矩阵计算器
'''
import open3d as o3d
from math import sqrt
import threading
import PSO_function
import DataLoader
from sko.tools import set_run_mode
import calibrateUtil_scipy as util
from sko.PSO import PSO
import numpy as np
from typing import Any
import sys
sys.path.append('./')


class pso():
    def __init__(self, args, psoFunc):
        self.args = args
        self.psoFunc = psoFunc

    # 获得以下距离作为一个小的评判标准
    def getDistance(self, Points, C, D):
        list = Points.tolist()

        print('AD', sqrt(pow(list[0]-D['x'], 2) +
                         pow(list[1]-D['y'], 2) + pow(list[2]-D['z'], 2)))
        print('AC', sqrt(pow(list[0]-C['x'], 2) +
                         pow(list[1]-C['y'], 2) + pow(list[2]-C['z'], 2)))
        print('DE', sqrt(pow(list[3]-D['x'], 2) +
                         pow(list[4]-D['y'], 2) + pow(list[5]-D['z'], 2)))
        print('AE', sqrt(pow(list[3]-list[0], 2) +
                         pow(list[4]-list[1], 2) + pow(list[5]-list[2], 2)))

    # 使用args参数运行pso
    def pso_run(self):
        pso = PSO(**self.args)
        # set_run_mode(args['func'], 'vectorization')
        set_run_mode(self.args['func'], 'cached')
        # pso.record_mode = True
        # print(pso.record_value['X']) # 打印记录的粒子位置信息
        # print(pso.record_value['Y']) # 打印记录的函数值
        pso.run()

        print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)

        return pso.gbest_x

    # 运行一次pso
    def caculate_pso_oneTime(self, data, countPose, type, countPoint=1):
        self.args['func'] = self.psoFunc.pso_function
        self.args['constraint_ueq'] = self.psoFunc.getConstraintList()

        self.psoFunc.setData(data)

        if type == 'point':
            # print(f"第{countPose}个位姿求解出的第{countPoint}个球心坐标\n", '='*30)
            bestPoint = self.pso_run()
            if np.array(bestPoint).ndim == 2:
                bestPoint = bestPoint[0]
            # self.getDistance(bestPoint, data[0], data[1])
        elif type == 'pose':
            print(f"第{countPose}个pso求解出的手眼矩阵\n", '='*30)
            bestPoint = self.pso_run()
            if np.array(bestPoint).ndim == 2:
                bestPoint = bestPoint[0]
            matrix = util.pose2Homo([bestPoint[:6]])[0]
            print(matrix)
        print('='*30)

        return bestPoint


class caculatePoint(object):
    def __init__(self, R, path, countPoint, psoArgs):
        '''
        Args:
            R (_type_): 法兰半径
            path (_type_): 标定数据根目录
            countPoint (_type_): 一个位姿计算几个球心
            psoArgs: pso参数
        '''
        self.R = R
        self.path = path
        self.countPoint = countPoint
        self.psoArgs = psoArgs

    # # 计算一个位姿对应的countPoint个球心
    # def caculate_point_onePose(self, data, countPose):
    #     for i in range(self.countPoint):
    #         threading.Thread(target=self.pso.caculate_pso_oneTime,
    #                          args=(data, countPose, type1="point", countPoint = i)).start()

    # # 利用多线程计算球心 - 无用
    # def caculate_point_Thread(self):
    #     dataLoader_1 = DataLoader.handleData_1(self.R, self.path)
    #     dataSet = dataLoader_1.getData()

    #     for i in range(0, len(dataSet)):
    #         threading.Thread(target=self.caculate_point_onePose,
    #                          args=(dataSet[i], i)).start()

    # 常规方法计算球心
    def caculate_point(self):
        '''PSO求解数组位姿对应的球心集

        Returns:
            dataSets[N][4]: [N][C, D, F, self.R]
            bestPoints[N][countPoint]: [N][countPoint][A, E]
        '''
        self.psoFunc = PSO_function.PSO_function_1()
        self.pso = pso(self.psoArgs, self.psoFunc)

        dataLoader = DataLoader.handleData_1(self.R, self.path)
        dataSets = dataLoader.getData()

        bestPoints = []
        for i in range(0, len(dataSets)):
            bestPoint_onePose = []
            for j in range(0, self.countPoint):
                bestPoint = self.pso.caculate_pso_oneTime(
                    dataSets[i], i, type='point', countPoint=j)
                if np.array(bestPoint).ndim == 2:
                    bestPoint = bestPoint[0]
                bestPoint_onePose.append([bestPoint[:3], bestPoint[3:]])

            bestPoints.append(bestPoint_onePose)

        return dataSets, bestPoints


class caculateHandEyeMatrix(object):
    def __init__(self, path, dataSets, bestPoints, psoArgs):
        '''初始化

        Args:
            path (_type_): 根目录
            dataSets (_type_): 已知标定数据
            bestPoints (_type_): 求解得到的球心
            psoArgs (_type_): pso求解参数
        '''
        self.path = path
        self.dataSets = dataSets
        self.bestPoints = bestPoints
        self.psoArgs = psoArgs

    # 计算球心所在平面
    def caculate_planes_M(self):
        planes_M = []

        for data in self.dataSets:
            # A, B 分别为点 A 和点 B 的坐标
            x1, y1, z1 = data[0]['x'], data[0]['y'], data[0]['z']
            x2, y2, z2 = data[1]['x'], data[1]['y'], data[1]['z']

            # 计算向量 AB
            CD = np.array([x2 - x1, y2 - y1, z2 - z1])

            F = np.array([data[2]['x'], data[2]['y'], data[2]['z']])

            # 构建平面方程

            planes_M.append(np.append(CD, -CD@F))

        return planes_M

    def caculate_pose(self):
        self.psoFunc = PSO_function.PSO_function_2()
        self.pso = pso(self.psoArgs, self.psoFunc)
        planes_M = self.caculate_planes_M()

        dataLoader = DataLoader.handleData_2(self.path)
        self.base2gripperList = dataLoader.getData()

        bestPose = self.pso.caculate_pso_oneTime(
            [self.base2gripperList, self.dataSets, self.bestPoints, planes_M], 1, type='pose', countPoint=1)

        return bestPose

    def showPointOnGripper(self, bestPose):
        util = PSO_function.PSO_Util
        paramsOnGripper, centersOnGripper = util.camera2gripper_point(
            self.dataSets, self.bestPoints, self.base2gripperList, bestPose)

        # print(paramsOnGripper, centersOnGripper)

        # 将点集转换为Open3D中的PointCloud对象
        pcd = o3d.geometry.PointCloud()
        for i in range(len(paramsOnGripper)):
            pcd.points.extend(o3d.utility.Vector3dVector(paramsOnGripper[i]))
            pcd.points.extend(o3d.utility.Vector3dVector(centersOnGripper[i]))

        # 可视化PointCloud对象
        o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    path = 'Data/LMI_gripper_calibrate'
    R = 20

    args = {'n_dim': 6, 'max_iter': 300, 'pop': 50,
            'lb': [-20, -20, 20, -20, -20, 20], 'ub': [20, 20, 70, 20, 20, 70], 'verbose': False}

    caculator = caculatePoint(R, path, countPoint=10, psoArgs=args)
    # caculate_point_Thread(R, path, 10)
    dataSets, bestPoints = caculator.caculate_point()
    print('\n'*20)
    # # 保存到txt文件
    # with open('dataSets.txt', 'w') as file:
    #     for row in dataSets:
    #         file.write(','.join(map(str, row)) + '\n')

    # with open('bestPoints.txt', 'w') as file:
    #     for row in bestPoints:
    #         file.write(','.join(map(str, row)) + '\n')

    # # 从txt文件加载数据
    # dataSets = []
    # with open('dataSets.txt', 'r') as file:
    #     for line in file:
    #         dataSets.append(list(map(int, line.strip().split(','))))

    # bestPoints = []
    # with open('bestPoints.txt', 'r') as file:
    #     for line in file:
    #         bestPoints.append(list(map(int, line.strip().split(','))))

    args = {'n_dim': 9, 'max_iter': 300, 'pop': 30,
            'lb': [550, 100, 350, -180, -180, -180, -50, -50, -50],
            'ub': [750, 300, 450, 180, 180, 180, 50, 50, 50], 'verbose': False}

    caculator_HEMatrix = caculateHandEyeMatrix(
        path, dataSets, bestPoints, args)
    caculator_HEMatrix.caculate_planes_M()
    bestPose = caculator_HEMatrix.caculate_pose()
    caculator_HEMatrix.showPointOnGripper(bestPose)
