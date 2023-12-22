'''
@ name: Austin
@ description: 待求解的最优化方程pso_function
               标定方法 - 标定板正交
@ Date: 2023-11-17 13:31:35
@ LastEditDate: 
'''
import calibrateByTest.data as data
import sys
import os
sys.path.append(os.path.abspath('./'))
import calibrateUtil_scipy as Util
import calibrateUtil as myUtil
import numpy as np


def getData_quaternion():
    target2camera_poseList = data.calPose
    gripper2base_poseList = data.toolPose

    target2camera_list, _, _ = Util.pose2Homo(target2camera_poseList)
    gripper2base_list, _, _ = Util.pose2Homo(gripper2base_poseList)
    return target2camera_list, gripper2base_list

target2camera_list, gripper2base_list = getData_quaternion()

# 这里有点问题
# 1.0604709843483415
# best_x is  [-9.96489501e-01  9.81030252e-02 -1.79529249e-01  1.60077183e-01
#   3.93119555e+01  1.00000000e+02  1.17505550e+03] best_y is [-6896.35173326]
def pso_function(particle):
    '''眼在手上, 利用
    gripper2base_list[i]@matrix@target2camera_list[i]=gripper2base_list[i+1]@matrix@target2camera_list[i+1]

    Args:
        w (_type_): _description_
        x (_type_): _description_
        y (_type_): _description_
        z (_type_): _description_
        a (_type_): _description_
        b (_type_): _description_
        c (_type_): _description_

    Returns:
        _type_: _description_
    '''    
    x, y, z, w, a, b, c = particle

    rotation_matrix = Util.quaternion2rotation_matrix([x, y, z, w])
    matrix = myUtil.R_T2HomogeneousMatrix(rotation_matrix, [a, b, c])
    
    sum_matrix = np.zeros(16).reshape(4, 4)
    
    length = len(target2camera_list)-1 if len(target2camera_list)%2==0 else len(target2camera_list)
    for i in range(1, length, 2):
        sum_matrix += gripper2base_list[i]@matrix@target2camera_list[i]
        sum_matrix -= gripper2base_list[i+1]@matrix@target2camera_list[i+1]
    print(sum_matrix.sum())
    return sum_matrix.sum()

def constraint_ueq_1(particle):
    return 0.9 - pow(particle[0], 2) + pow(particle[1], 2) + pow(particle[2], 2) + pow(particle[3], 2)

def constraint_ueq_2(particle):
    return pow(particle[0], 2) + pow(particle[1], 2) + pow(particle[2], 2) + pow(particle[3], 2) - 1.1

