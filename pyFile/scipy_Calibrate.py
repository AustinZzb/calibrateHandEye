'''
@ name: Austin
@ description: 一些现成的方法以及对手眼标定做误差分析
@ Date: 2023-09-21 17:13:50
@ LastEditDate: 
'''
from turtle import width
import prettytable
from scipy.spatial.transform import Rotation as R
import numpy as np
import table


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


def testCalibrateError_Point(camera2gripper, target2camera_list, gripper2base_list, isExtend = True, floatLen = 2):
    '''对使用空间定点求出的手眼矩阵进行误差分析\n
    将第一组点假设为空间点的真值，进行误差估计

    Args:
        camera2gripper (ndarray): 手眼矩阵\n
        target2camera_list (list): 目标到相机的坐标转换list\n
        gripper2base_list (list): 末端到基底的坐标转换list\n
        isExtend (bool): 是否对详细数据进行展开(默认值为True)\n
        floatLen (int): 数据保存小数点后几位，默认值为2
    '''

    if floatLen < 0:
        floatLen = 2
    
    TruePoseMatrix = camera2gripper @ target2camera_list[0] @ gripper2base_list[0]
    TruePose = Matrix2Pose(TruePoseMatrix)
    '''以第一点当作真实点，去估算手眼标定误差
    '''

    poseList = []
    error_list = []
    errorAbs = []
    pTable = prettytable.PrettyTable()
    pTable.field_names = ['', 'x(误差)', 'y(误差)', 'z(误差)', 'rx(误差)', 'ry(误差)', 'rz(误差)']

    for i in range(0, len(target2camera_list)):
        target2camera = target2camera_list[i]
        gripper2base = gripper2base_list[i]

        currentPose = Matrix2Pose(camera2gripper @ target2camera @ gripper2base)

        poseList.append(currentPose)
        error_list.append(currentPose - TruePose)
        

        # 对误差数据取绝对值
        tempErrorAbs = []
        for j in range(0, len(error_list[i])):
            tempErrorAbs.append(abs(error_list[i][j])) 
        errorAbs.append(tempErrorAbs)
        
        if isExtend:            
            tempPose = list(poseList[i])
            tempError = list(error_list[i])
            
            tempList = ['No.'+str(i+1)]
            for pose, error in zip(tempPose, tempError):
                tempList.append(str(round(pose, floatLen)) + '(' + str(round(error, floatLen)) + ')' )

            pTable.add_row(tempList)
    
    errorArray = np.array(errorAbs)

    tempList_1 = list(np.mean(errorArray, axis=0))
    tempList_1.insert(0, '平均值')
    tempList_2 = list(np.amax(errorArray, axis=0))
    tempList_2.insert(0, '最大值')
    tempList_3 = list(np.amin(errorArray, axis=0))
    tempList_3.insert(0, '最小值')
    tempList_4 = list(np.var(errorArray, axis=0))
    tempList_4.insert(0, '方差')

    pTable.add_row(tempList_1)
    pTable.add_row(tempList_2)
    pTable.add_row(tempList_3)
    pTable.add_row(tempList_4)
            
    print(pTable)





