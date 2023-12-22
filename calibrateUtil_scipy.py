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
import calibrateUtil as myUtil

def pose2Homo(poseList):
    '''传入点列表返回其次变换矩阵

    Args:
        poseList (list): _description_
    
    Returns:
        Homo_matrix: 齐次变换矩阵
        R_matrix: 旋转矩阵
        T_matrix: 平移矩阵
    '''
    T_matrix = [] # 末端到基底的平移矩阵
    R_matrix = [] 
    Homo_matrix = []

    for itemPose in poseList:
        # 获取欧拉角并转换为旋转矩阵
        R_itemPose = R.as_matrix(R.from_euler("xyz", itemPose[3:]))
        R_matrix.append(R_itemPose)
        T_itemPose = itemPose[:3]
        T_matrix.append(T_itemPose)
        Homo_itemCalPose = myUtil.R_T2HomogeneousMatrix(R_itemPose, T_itemPose)  
        Homo_matrix.append(Homo_itemCalPose)
    
    return Homo_matrix, R_matrix, T_matrix

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
    '''四元数转旋转矩阵

    Args:
        quaternion (list): [x, y, z, w]

    Returns:
        rotation_matrix(ndarray): 旋转矩阵
    '''
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

def is_rotation_matrix(matrix):
    # 检查行列式是否为1或-1
    if np.abs(np.linalg.det(matrix) - 1) > 1e-6:
        return False
    # 检查逆矩阵是否等于转置矩阵
    if not np.allclose(np.linalg.inv(matrix), np.transpose(matrix)):
        return False
    return True

def normalize_quaternion(q):
    """
    将四元数归一化为单位四元数
    :param q: 四元数 [w, x, y, z]
    :return: 归一化后的四元数 [w, x, y, z]
    """
    norm = np.linalg.norm(q)
    if norm == 0:
        return q
    return q / norm

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
    print('x, y, z误差的平均值：', (tempList_1[1]+tempList_1[2]+tempList_1[3])/3)


# # 末端到基底的齐次矩阵、旋转矩阵和平移矩阵
# Homo_gripper2base, R_gripper2base, T_gripper2base = pose2Homo(data.toolPose)
# # 标定物到相机的齐次矩阵、旋转矩阵和平移矩阵
# Homo_target2cam, R_target2cam, T_target2cam = pose2Homo(data.calPose)

# print('{', end='')
# for i in range(0, 13):
#     print('{', end='')
#     for j in range(0, 4):
#         print('{', end='')
#         for k in range(0, 4):
#             print(Homo_target2cam[i][j][k], ',', end='')
#         print('},', end='')
#     print('},', end='')
# print('}', end='')




