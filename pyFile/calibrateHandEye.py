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

R_gripper2base = [] # 末端到基底的旋转矩阵
T_gripper2base = [] # 末端到基底的平移矩阵
R_target2cam = []   # 标定物到相机的旋转矩阵
T_target2cam = []   # 标定物到相机的平移矩阵
Homo_R_gripper2base = [] # 齐次矩阵
Homo_target2cam = []

for itemCalPose, itemToolPose in zip(data.calPose, data.toolPose):
    # 获取欧拉角并转换为旋转矩阵
    R_itemCalPose = Rota.as_matrix(Rota.from_euler("xyz", itemCalPose[3:]))
    R_itemToolPose = Rota.as_matrix(Rota.from_euler("xyz", itemToolPose[3:]))
    
    R_target2cam.append(R_itemCalPose)
    R_gripper2base.append(R_itemToolPose)
    
    T_target2cam.append(itemCalPose[:3])
    T_gripper2base.append(itemToolPose[:3])
    