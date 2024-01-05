'''
Author: Austin 1158680540@qq.com
Date: 2023-12-20 15:14:53
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-04 17:18:53
FilePath: \calibrateHandEye\calibrateByGripper\run.py
Description: 球心 & 手眼矩阵计算器
'''
from typing import Any
import numpy as np
from sko.PSO import PSO
from sko.tools import set_run_mode
import DataLoader
import PSO_function
import threading
from math import sqrt


class caculatePoint(object):
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

    def pso_run(self, args):
        pso = PSO(**args)
        # set_run_mode(args['func'], 'vectorization')
        set_run_mode(args['func'], 'cached')
        # pso.record_mode = True
        # print(pso.record_value['X']) # 打印记录的粒子位置信息
        # print(pso.record_value['Y']) # 打印记录的函数值
        pso.run()

        print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)

        return pso.gbest_x

    # 计算一次球心
    def caculate_point_onePoint(self, data, countPose, countPoint):
        args = {'n_dim': 6, 'max_iter': 200, 'pop': 300,
                'lb': [-20, -20, 20, -20, -20, 20], 'ub': [20, 20, 70, 20, 20, 70], 'verbose': False}

        temPsoFunc = PSO_function.PSO_function_1()

        args['func'] = temPsoFunc.pso_function
        args['constraint_ueq'] = temPsoFunc.getConstraintList()

        temPsoFunc.setData(data)

        bestPoint = self.pso_run(args)

        print(f"第{countPose}个位姿求解出的第{countPoint}个球心坐标\n", '='*30)
        self.getDistance(bestPoint, data[0], data[1])
        print('='*30)

        return bestPoint

    # 计算一个位姿对应的countPoint个球心
    def caculate_point_onePose(self, data, countPose, countPoint):
        for i in range(countPoint):
            threading.Thread(target=self.caculate_point_onePoint,
                             args=(data, countPose, countPoint)).start()

    # 利用多线程计算球心 - 无用
    def caculate_point_Thread(self, R, path, countPoint):
        dataLoader_1 = DataLoader.handleData_1(R, path)
        dataSet = dataLoader_1.getData()

        for i in range(0, len(dataSet)):
            threading.Thread(target=self.caculate_point_onePose,
                             args=(dataSet[i], i, countPoint)).start()

    # 常规方法计算球心

    def caculate_point(self, R, path, countPoint):
        '''PSO求解数组位姿对应的球心集

        Args:
            R (_type_): 法兰半径
            path (_type_): 标定数据根目录
            countPoint (_type_): 一个位姿计算几个球心

        Returns:
            dataSets[N][4]: [N][C, D, F, self.R]
            bestPoints[N][countPoint]: [N][countPoint][A, E]
        '''
        dataLoader_1 = DataLoader.handleData_1(R, path)
        dataSets = dataLoader_1.getData()

        bestPoints = []
        for i in range(0, len(dataSets)):
            bestPoint_onePose = []
            for j in range(0, countPoint):
                bestPoint = self.caculate_point_onePoint(dataSets[i], i, j)
                bestPoint_onePose.append([bestPoint[:3], bestPoint[3:]])

            bestPoints.append(bestPoint_onePose)

        return dataSets, bestPoints


class caculateHandEyeMatrix(object):
    def __init__(self, dataSets, bestPoints):
        self.dataSets = dataSets
        self.bestPoints = bestPoints

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
        print(planes_M)
        return planes_M


if __name__ == '__main__':
    path = 'Data/LMI_gripper_calibrate'
    R = 20

    caculator = caculatePoint()
    # caculate_point_Thread(R, path, 10)
    dataSets, bestPoints = caculator.caculate_point(R, path, 10)

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

    caculator_HEMatrix = caculateHandEyeMatrix(dataSets, bestPoints)
    caculator_HEMatrix.caculate_planes_M()
