'''
@ name: Austin
@ description: 使用scipy.spatial.transform.Rotation和Tsai求解手眼矩阵
@ Date: 2023-09-22 15:36:23
@ LastEditDate: 
'''
import sys
import os
sys.path.append(os.path.abspath('./'))

from re import T
import cv2
import data

import calibrateUtil as myUtil
import calibrateUtil_scipy as Util
import TSAI

# 末端到基底的齐次矩阵、旋转矩阵和平移矩阵
Homo_gripper2base, R_gripper2base, T_gripper2base = Util.pose2Homo(data.toolPose)
# 标定物到相机的齐次矩阵、旋转矩阵和平移矩阵
Homo_target2cam, R_target2cam, T_target2cam = Util.pose2Homo(data.calPose)
print(TSAI.TSAI(data.toolPose, data.calPose))
print("="*10)

R_cam2gripper, T_cam2gripper = cv2.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam)
print(T_cam2gripper.reshape(1, 3), Util.rotation_matrix2euler(R_cam2gripper))
Homo_cam2gripper = myUtil.R_T2HomogeneousMatrix(R_cam2gripper, T_cam2gripper)

print("\n手眼矩阵：\n", Homo_cam2gripper)
print('\nTSAI误差分析')
Util.testCalibrateError_Point(Homo_cam2gripper, Homo_target2cam, Homo_gripper2base, isExtend=True, floatLen = 2)

print(Util.is_rotation_matrix(Homo_cam2gripper[:3, :3]))

# PSO算法
list = [-0.102133,0.615758,0.0628547,0.8,-0.133163,0.0527855,-0.33238]
Homo_cam2gripper = myUtil.R_T2HomogeneousMatrix(Util.quaternion2rotation_matrix(Util.normalize_quaternion(list[:4])), list[4:])
print("\n手眼矩阵：\n", Homo_cam2gripper)
print('\nPSO误差分析')
Util.testCalibrateError_Point(Homo_cam2gripper, Homo_target2cam, Homo_gripper2base, isExtend=True, floatLen = 2)

print(Util.is_rotation_matrix(Homo_cam2gripper[:3, :3]))