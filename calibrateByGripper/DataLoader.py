'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 16:10:02
LastEditors: Austin 58591300+Justin-zzb@users.noreply.github.com
LastEditTime: 2023-12-22 17:53:57
FilePath: \calibrateHandEye\calibrateByGripper\DataLoader.py
Description: 超参数全部在DataLoader, 不能出现在pso_function
'''
import os
import sys
sys.path.append('./')
import calibrateUtil as myUtil
import calibrateUtil_scipy as util
import open3d as o3d
import numpy as np
from math import sqrt



class handleData_1():
    def __init__(self, R, C, D):
        self.R = R
        self.C = C
        self.D = D

    def getData(self):
        robot_pose = []
        line = []

        # for root, dirs, files in os.walk(rootDir):

        # source = o3d.geometry.PointCloud()
        # m1 = np.loadtxt('./Data/LMI_gripper_calibrate/replay_128136_2023-12-14 (3).txt', delimiter=",")
        # source.points = o3d.utility.Vector3dVector(m1)

        # 获取坐标

        # 提取弦两端点(C[x1, 0, z1], D[x2, 0, z2]) -> 中点(F[x, 0, z])
        # C = {'x':4.7, 'y':0, 'z':20}
        # D = {'x': 0, 'y': 0, 'z': 20}
        F = {'x':(self.C['x']+self.D['x'])/2, 'y':0, 'z':(self.C['z']+self.D['z'])/2}
        # 根据三个勾股定理求解球心(A[x, y, z'])
        # A = {'x': self.F['x']}

        # 点D和点F不一定在一条直线
        DF = sqrt(pow(self.D['z']-F['z'], 2) + pow(self.D['x']-F['x'], 2))
        # AF = sqrt(pow(self.R, 2) - pow(DF, 2))

        return self.C, self.D, self.R, F, DF


def getData_2(rootDir):
    path = ""

    for root, dirs, files in os.walk(rootDir):
        path = os.path.join(rootDir, files[0])
        break

    lines = open(path, 'r').readlines()

    poseList = []
    for line in lines:
        pose = line.strip().split(' ')[:-1]
        pose = [np.double(temp) for temp in pose]

        poseList.append(pose)

    base2gripper, _, _ = util.pose2Homo(poseList)

    return base2gripper
