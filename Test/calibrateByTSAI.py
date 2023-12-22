'''
@ name: Austin
@ description: 使用scipy.spatial.transform.Rotation和Tsai求解手眼矩阵
@ Date: 2023-09-22 15:36:23
@ LastEditDate: 
'''
from re import T
import cv2
import data
import calibrateUtil as myUtil
import calibrateUtil_scipy as Util
import numpy as np

def TSAI():
    '''使用TSAI求解手眼矩阵, 返回
       齐次变换矩阵形式和四元数形式的手眼矩阵

    @return: Homo_cam2gripper, quaternion_cam2gripper
        
    '''

    # 末端到基底的齐次矩阵、旋转矩阵和平移矩阵
    Homo_gripper2base, R_gripper2base, T_gripper2base = Util.pose2Homo(data.toolPose)
    # 标定物到相机的齐次矩阵、旋转矩阵和平移矩阵
    Homo_target2cam, R_target2cam, T_target2cam = Util.pose2Homo(data.calPose)

    R_cam2gripper, T_target2cam = cv2.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam)

    Homo_cam2gripper = myUtil.R_T2HomogeneousMatrix(R_cam2gripper, T_target2cam)

    print("\n手眼矩阵: \n", Homo_cam2gripper)

    # 旋转矩阵转换为四元数
    quaternion = Util.rotation_matrix2quaternion(Homo_cam2gripper[:3,:3])
    translate = Homo_cam2gripper[:3,-1].reshape(1, 3)[0]

    quaternion_cam2gripper = np.concatenate((quaternion, translate))
    print("\n四元数: \n", quaternion_cam2gripper)

    return Homo_cam2gripper, quaternion_cam2gripper

TSAI()