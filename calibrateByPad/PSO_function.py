'''
@ name: Austin
@ description: 待求解的最优化方程pso_function
               标定方法 - 标定板正交
@ Date: 2023-11-17 13:31:35
@ LastEditDate: 
'''
import sys
import os
sys.path.append(os.path.abspath('./'))
import random
import calibrateUtil_scipy as Util
import calibrateUtil as myUtil
import numpy as np


def getData_orthogonal(lineCount=1, pointCount=12):
    targetVector_h, targetVector_v, gripper2base_h, gripper2base_v = [], [], [], []

    file_position = open("./Data/Line/position.txt", "r")
    lines_position = file_position.readlines()
    lines_position = [line.replace('\n','').split(" ") for line in lines_position]
    
    for i in range(1, lineCount+1):
        for j in range(1, 3):
            file = open("./Data/Line/"+str(i)+"."+str(j)+".txt", "r")
            lines = file.readlines()
            lines = [line for line in lines if "-10000.0000" not in line]

            length = len(lines)
            tempArr = []

            for k in range(0, pointCount):
                a = random.randint(0, length)
                b = (random.randint(0, length)+a)%length
                
                line_a_x, line_a_z = lines[a].replace('\n','').split("\t")
                line_b_x, line_b_z = lines[b].replace('\n','').split("\t") 

                tempArr.append([[np.double(line_a_x), 0, np.double(line_a_z), 1], [np.double(line_b_x), 0, np.double(line_b_z), 1]])
                
            if (j == 1):
                targetVector_h.append(tempArr)
            else:
                targetVector_v.append(tempArr)
            
        gripper2base_h.append([np.double(num) for num in lines_position[i*2-1]])
        gripper2base_v.append([np.double(num) for num in lines_position[i*2]])       
    
    gripper2base_h, _, _ = Util.pose2Homo(gripper2base_h)
    gripper2base_v, _, _ = Util.pose2Homo(gripper2base_v)
    
    return targetVector_h, targetVector_v, gripper2base_h, gripper2base_v           


# 标定板上的n组两两垂直的向量, 以及n组机器人位姿
targetVector_h, targetVector_v, gripper2base_h, gripper2base_v = getData_orthogonal(lineCount=4)

def pso_function(particle):
    '''眼在手外, 利用标定板上相互正交的两个边上的点集的向量相互正交
    '''
    x, y, z, w, a, b, c = particle
    
    rotation_matrix = Util.quaternion2rotation_matrix([x, y, z, w])
    matrix = myUtil.R_T2HomogeneousMatrix(rotation_matrix, [a, b, c])
    
    sum = 0
    for i in range(len(gripper2base_h)):
        for j in range(len(targetVector_h)):
            vec_a = (np.linalg.inv(gripper2base_h[i])@matrix@targetVector_h[i][j][0] - np.linalg.inv(gripper2base_h[i])@matrix@targetVector_h[i][j][1])
            vec_b = (np.linalg.inv(gripper2base_v[i])@matrix@targetVector_v[i][j][0] - np.linalg.inv(gripper2base_v[i])@matrix@targetVector_v[i][j][1])
            temp = vec_a.T @ vec_b
            
            # 因为合理点的坐标维度是四位，最后一位是为了方便计算，最后两个向量的点击需要-1
            sum += abs(temp)
            # print(math.acos(temp/(np.linalg.norm(vec_a) * np.linalg.norm(vec_b))))
    return sum


def constraint_ueq_1(particle):
    return 0.9 - pow(particle[0], 2) + pow(particle[1], 2) + pow(particle[2], 2) + pow(particle[3], 2)

def constraint_ueq_2(particle):
    return pow(particle[0], 2) + pow(particle[1], 2) + pow(particle[2], 2) + pow(particle[3], 2) - 1.1

