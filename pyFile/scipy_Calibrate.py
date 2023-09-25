'''
@ name: Austin
@ description: 
@ Date: 2023-09-21 17:13:50
@ LastEditDate: 
'''
from scipy.spatial.transform import Rotation as R

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

if __name__ == '__main__':
    # 四元数=>欧拉角
    quaternion = [0.71934025092983234, -1.876085535681999e-06, -3.274841213980097e-08, -0.69465790385533299]
    euler = quaternion2euler(quaternion) # [-9.20000743e+01  1.52039496e-04 -1.52039496e-04]
    print(f'euler: {euler}')
    
    # 四元数=>旋转矩阵
    rotation_matrix = quaternion2rotation_matrix(quaternion)
    print(f'rotation_matrix: {rotation_matrix}')
    
    # 欧拉角=>四元数
    quaternion = euler2quaternion(euler)
    print(f'quaternion: {quaternion}') # [-7.19340251e-01  1.87608554e-06  3.27484122e-08  6.94657904e-01]
    
    # 欧拉角=>旋转矩阵
    rotation_matrix = euler2rotation(euler)
    print(f'rotation_matrix: {rotation_matrix}')
    
    # 旋转矩阵=>欧拉角
    euler = rotation_matrix2euler(rotation_matrix)
    print(f'euler: {euler}')
    
    # 旋转矩阵=>四元数
    quaternion = rotation_matrix2quaternion(rotation_matrix)
    print(f'quaternion: {quaternion}')
