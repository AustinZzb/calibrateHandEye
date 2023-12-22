'''
@ name: Austin
@ description: 自己造的一些关于手眼标定的轮子
@ Date: 2023-09-21 09:34:00
@ LastEditDate: 
'''
import math
import numpy as np

def R_T2HomogeneousMatrix(R, T) -> np.array :
    '''
        将旋转矩阵和平移向量转换为齐次矩阵
        @params R:旋转矩阵 T:平移矩阵
        @return 其次变换矩阵
    '''
    
    tempMat = np.hstack((R, np.array(T).reshape(3, 1)))
    '''将R和T水平拼接, 拼接列向量'''

    resultMat = np.vstack((tempMat, [[0.0, 0.0, 0.0, 1.0]]))
    '''将R和T垂直拼接, 拼接行向量'''
    
    return resultMat

def HomogeneousMatrix2RT(Mat) -> (np.array, np.array) :
    '''将齐次变换矩阵拆分为旋转矩阵和平移矩阵

    Args:
        Mat (ndarray): 齐次变换矩阵

    Returns:
        np.array: 旋转矩阵
        np.array: 平移向量
    '''
    return Mat[:3, :3], Mat[:3, -1]  
    
def eulerAngleToRotationMatrix(eulerAngle, seq) -> np.array:
    '''将欧拉角转换为旋转矩阵（此处默认传入的eulerAngle的顺序是rx, ry, rz）

    Args:
        eulerAngle (ndarray||list): 欧拉角矩阵或者列表
        seq (str):  欧拉角顺序

    Returns:
        rotMat(np.array): 旋转矩阵
    '''
    if eulerAngle.ndim == 2:
        # 如果eulerAngle是1*3矩阵
        eulerAngle = eulerAngle[0]
    
    # 转换为弧度制
    eulerAngle = eulerAngle * (np.pi / 180)

    # 分别求解其正弦值和余弦值
    rxs, rys, rzs = [math.sin(item) for item in eulerAngle]
    rxc, ryc, rzc = [math.cos(item) for item in eulerAngle]

    # 各方向上的旋转矩阵
    Mat_x = np.array([[1, 0, 0],
                        [0, rxc, -rxs],
                        [0, rxs, rxc]])
    Mat_y = np.array([[ryc, 0, rys],
                        [0, 1, 0],
                        [-rys, 0, ryc]])
    Mat_z = np.array([[rzc, -rzs, 0],
                        [rzs, rzc, 0],
                        [0, 0, 1]])
    
    MatXYZ = {"x": Mat_x, "y": Mat_y, "z": Mat_z}

    # 根据欧拉角顺序求解旋转矩阵
    seqList = list(seq)
    rotMat = np.dot(MatXYZ[seqList[2]], MatXYZ[seqList[1]], MatXYZ[seqList[0]]) 

    return rotMat

def isRotaMatrix(Mat) -> bool:
    '''判断这个矩阵是否为旋转矩阵，即旋转矩阵是否有解(欧拉角万向锁之类的)
    \n判断方式: 旋转矩阵 @ 旋转矩阵的逆 = 单位阵

    Args:
        Mat (ndarray): 求解得出的旋转矩阵

    Returns:
        isRotMat (bool): 是否是旋转矩阵
    '''
    RoMat = Mat[:3, :3] # 提取前3行前3列





    