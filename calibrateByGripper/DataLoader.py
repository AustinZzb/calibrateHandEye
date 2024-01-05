'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 16:10:02
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-03 14:47:37
FilePath: \calibrateHandEye\calibrateByGripper\DataLoader.py
Description: 超参数全部在DataLoader, 不能出现在pso_function
'''
import os
import sys
sys.path.append('./')
import calibrateUtil as myUtil
import calibrateUtil_scipy as util
import open3d as o3d
import open3dUtil as o3dUtil
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


'''
根据根目录
读取**全部**机器人位姿或激光线坐标
'''
class readData():
    
    def __init__(self, rootDir):
        self.rootDir = rootDir

    def getPath(self, isPose):
        self.paths = []

        for root, dirs, files in os.walk(self.rootDir):
            if isPose == True:
                self.paths.append(os.path.join(root, files[0]))
                break
            elif len(dirs) == 0: # 没有dirs只有files表示是激光线的点
                for file in files:
                    self.paths.append(os.path.join(root, file))

    def getPose(self):
        '''读取机器人位姿数据

        Returns:
            list[N][6]: 机器人位姿
        '''
        self.getPath(True)
        lines = open(self.paths[0], 'r').readlines()

        poseList = []
        for line in lines:
            pose = line.strip().split(' ')[:-1]
            pose = [np.double(temp) for temp in pose]

            poseList.append(pose)

        return poseList
    
    def getPoint(self):
        '''速度有些慢

        Args:
            R (_type_): _description_

        Returns:
            ndarray[n][2][3]: _description_
        '''
        self.getPath(False)

        pointList = []
        for path in self.paths:
            lines = open(path, 'r').readlines()
            points = []

            for line in lines:
                points.append([np.double(str) for str in line.strip().split(',')])
                
            
            points = np.array(points)
            points = points[np.argsort(points[:, 0])]
            np.set_printoptions(threshold=np.inf)

            # 获取第一列平均值
            mean_z = np.mean(points[:, 2])
            # 剔除小于平均值的行
            filtered_line = points[points[:, 2] >= mean_z]
            # pcd = o3dUtil.createPCD(filtered_line)
            # o3dUtil.showPointCloud(pcd)

            # 从中间向两边扩散寻找端点
            meanIndex = int(filtered_line.shape[0]/2)
            leftIndex, rightIndex = meanIndex, meanIndex+1
            while True:
                if abs(filtered_line[leftIndex][0]-filtered_line[leftIndex+1][0]) > 2:
                    rightIndex = leftIndex+1
                    break
                elif abs(filtered_line[rightIndex][0]-filtered_line[rightIndex-1][0]) > 2:
                    leftIndex = rightIndex-1
                    break
                else:
                    leftIndex -= 1
                    rightIndex += 1            

            # 拟合直线
            xArr = filtered_line[:, 0]
            zArr = filtered_line[:, 2]

            coeffs = np.polyfit(xArr, zArr, 1)
            a = coeffs[0]
            b = coeffs[1]

            # 绘制数据和拟合直线
            # plt.scatter(xArr, zArr)
            # plt.plot(xArr, a * xArr + b, color='red')
            # plt.yticks(np.arange(-50, 50, step=0.05))
            # plt.show()
            # print(f'{a} * x + {b}')

            # 为减小误差, 将边缘点的x值代入直线求得直线上的边缘点
            pointList.append([[filtered_line[leftIndex, 0], 0, a*filtered_line[leftIndex, 0]+b], 
                              [filtered_line[rightIndex, 0], 0, a*filtered_line[rightIndex, 0]+b]]) 

        return pointList


'''
1.初始化需要法兰半径
2.处理激光线数据获取弦端点值
'''
class handleData_1():
    def __init__(self, R, path):
        self.R = R

        reader = readData(path)
        self.pointList = reader.getPoint()

    def getData(self):
        '''
            返回dataSets = [C, D, F, self.R]
        '''
        dataSet = []

        for pointCouple in self.pointList:
            C = dict([('x', pointCouple[0][0]), ('y', pointCouple[0][1]), ('z', pointCouple[0][2])])
            D = dict([('x', pointCouple[1][0]), ('y', pointCouple[1][1]), ('z', pointCouple[1][2])])

            # 求解弦中点坐标
            F = {'x':(C['x']+D['x'])/2, 'y':0, 'z':(C['z']+D['z'])/2}

            dataSet.append([C, D, F, self.R])

        return dataSet


'''
处理机器人位姿
将欧拉角形式的位姿转换为齐次矩阵且求逆[因为原始数据是E->B, 实际需要B->E]
'''
class handleData_2():
    def __init__(self, rootDir):
        reader = readData(rootDir)
        self.poseList = reader.getPose()

    def getData(self):
        '''
        Returns:
            list[N][4][4]: 各位姿对应的齐次变换矩阵
        '''
        base2gripperList, _, _ = util.pose2Homo(self.poseList)

        return base2gripperList


if __name__ == '__main__':
    # handle = handleData_1(20, 'Data/LMI_gripper_calibrate')
    # print(handle.getData())
    handle = handleData_2('Data/LMI_gripper_calibrate')
    print(handle.getData())
