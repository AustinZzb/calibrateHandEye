'''
@ name: Austin
@ description: 使用scipy.spatial.transform.Rotation和Tsai求解手眼矩阵
@ Date: 2023-09-22 15:36:23
@ LastEditDate: 
'''
import math
from re import T
import cv2
import numpy as np
import data
from scipy.spatial.transform import Rotation as Rota
import calibrateHandEyeUtil as myUtil
import scipy_Calibrate as Util

R_gripper2base = [] # 末端到基底的旋转矩阵
T_gripper2base = [] # 末端到基底的平移矩阵
R_target2cam = []   # 标定物到相机的旋转矩阵
T_target2cam = []   # 标定物到相机的平移矩阵
Homo_gripper2base = [] # 齐次矩阵
Homo_target2cam = []

for itemCalPose, itemToolPose in zip(data.calPose, data.toolPose):
    # 获取欧拉角并转换为旋转矩阵
    R_itemCalPose = Rota.as_matrix(Rota.from_euler("xyz", itemCalPose[3:]))
    R_itemToolPose = Rota.as_matrix(Rota.from_euler("xyz", itemToolPose[3:]))
    
    R_target2cam.append(R_itemCalPose)
    R_gripper2base.append(R_itemToolPose)
    
    T_itemCalPose = itemCalPose[:3]
    T_itemToolPose = itemToolPose[:3]

    T_target2cam.append(T_itemCalPose)
    T_gripper2base.append(T_itemToolPose)

    Homo_itemCalPose = myUtil.R_T2HomogeneousMatrix(R_itemCalPose, T_itemCalPose)
    Homo_itemToolPose = myUtil.R_T2HomogeneousMatrix(R_itemToolPose, T_itemToolPose)
    
    Homo_target2cam.append(Homo_itemCalPose)
    Homo_gripper2base.append(Homo_itemToolPose)

R_cam2gripper, T_target2cam = cv2.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam)

Homo_cam2gripper = myUtil.R_T2HomogeneousMatrix(R_cam2gripper, T_target2cam)

print("\n手眼矩阵：\n", Homo_cam2gripper)
print('\n误差分析')
Util.testCalibrateError_Point(Homo_cam2gripper, Homo_target2cam, Homo_gripper2base, isExtend=True, floatLen = 2)
