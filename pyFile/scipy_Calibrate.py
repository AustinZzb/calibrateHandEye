'''
@ name: Austin
@ description: 
@ Date: 2023-09-21 17:13:50
@ LastEditDate: 
'''
from turtle import width
from scipy.spatial.transform import Rotation as R
import numpy as np

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def euler2quaternion(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    quaternion = r.as_quat()
    return quaternion

def euler2rotation(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    rotation_matrix = r.as_matrix()
    return rotation_matrix

def quaternion2rotation_matrix(quaternion):
    r = R.from_quat(quaternion)
    rotation_matrix = r.as_matrix()
    return rotation_matrix

def rotation_matrix2euler(rotation_matrix):
    r = R.from_matrix(rotation_matrix)
    euler = r.as_euler('xyz', degrees=True)
    return euler
    

def rotation_matrix2quaternion(rotation_matrix):
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()
    return quaternion

def Matrix2Pose(Matrix) -> np.array:
    '''将一个其次变换矩阵转换为(x, y, z, rx, ry, rz)的形式

    Args:
        Matrix2Pose (ndarray): 其次变换矩阵

    Returns:
        np.array: 返回[x, y, z, rx, ry, rz]
    '''
    euler = R.from_matrix(Matrix[:3, :3]).as_euler('xyz')
    Pose = np.concatenate((Matrix[:3, 3], euler), axis=0)

    return Pose


def testCalibrateError_Point(camera2gripper, target2camera_list, gripper2base_list, isExtend = True):
    '''对使用空间定点求出的手眼矩阵进行误差分析\n
    将第一组点假设为空间点的真值，进行误差估计

    Args:
        camera2gripper (ndarray): 手眼矩阵\n
        target2camera_list (list): 目标到相机的坐标转换list\n
        gripper2base_list (list): 末端到基底的坐标转换list\n
        isExtend (bool): 是否对详细数据进行展开
    '''
    
    TruePoseMatrix = camera2gripper @ target2camera_list[0] @ gripper2base_list[0]
    TruePose = Matrix2Pose(TruePoseMatrix)
    '''以第一点当作真实点，去估算手眼标定误差
    '''

    poseList = []
    error_list = []

    for i in range(1, len(target2camera_list)):
        target2camera = target2camera_list[i]
        gripper2base = gripper2base_list[i]

        currentPose = Matrix2Pose(camera2gripper @ target2camera @ gripper2base)

        poseList.append(currentPose)
        error_list.append(currentPose - TruePose)

    if isExtend:
        print('%20s'*6 % ('x(误差)', 'y(误差)', 'z(误差)', 'rx(误差)', 'ry(误差)', 'rz(误差)'))

        print('%-s' % 'No.1', end="")
        for item in TruePose:
            print('%20f (0)' % item, end="")

        for i in range(0, len(poseList)):
            print('\nNo.%-d' % (i+2), end="")
            for pose, error in zip(poseList[i], error_list[i]):
                print('%20f (%f)' % (item, error), end="")
            print()

    errorArray = np.array(error_list)

    print('平均值   ', end='')
    print(np.mean(errorArray, axis=0))
    print('最大值   ', end='')
    print(np.amax   (errorArray, axis=0))
    print('最小值   ', end='')
    print(np.amin(errorArray, axis=0))
    print('方差     ', end='')
    print(np.var(errorArray, axis=0))

