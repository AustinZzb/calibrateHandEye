'''
Author: Austin 1158680540@qq.com
Date: 2023-11-17 10:56:33
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-12-19 15:20:23
FilePath: \calibrateHandEye\pyFile\ransan_Line.py
Description: 使用手眼矩阵将标定板上的点坐标系从相机移至基坐标下
'''
import calibrateByPad.PSO_function as data
import open3d as o3d
import numpy as np
import calibrateUtil as myUtil
import calibrateUtil_scipy as Util

pints_2d = []

def getPointCloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def reverseTarget2gripper(lineGroup, position, matrix):
    points = []

    for pointGroup in lineGroup:
        a = np.linalg.inv(position) @ matrix @ pointGroup[0]
        b = np.linalg.inv(position) @ matrix @ pointGroup[1]

        points.append(a[:3])
        points.append(b[:3])
        pints_2d.append(a[:2])
        pints_2d.append(b[:2])
        print(a[:3], '\n' , b[:3])

    return points

def getPointsOnGripper():
    points = []

    rotation_matrix = Util.quaternion2rotation_matrix([9.99996433e-01,-1.06653183e-01 , 2.73910091e-01, -1.15674414e-01])
    matrix = myUtil.R_T2HomogeneousMatrix(rotation_matrix, [-1.35728218e+01  ,1.86976250e+02  ,1.17345465e+03])

    for line_h, line_v, position_h, position_v in zip(targetVector_h, targetVector_v, gripper2base_h, gripper2base_v):
        point_h = reverseTarget2gripper(line_h, position_h, matrix)
        point_v = reverseTarget2gripper(line_v, position_v, matrix)

        points.append(point_h)
        points.append(point_v)

    return points

targetVector_h, targetVector_v, gripper2base_h, gripper2base_v = data.getData_orthogonal(lineCount=5, pointCount=150)
points = getPointsOnGripper()

pcds = []
for line in points:
    pcd = getPointCloud(line)
    pcds.append(pcd)

# import matplotlib.pyplot as plt

# # x和y列表数据
# x = [num[0] for num in pints_2d]
# y = [num[1] for num in pints_2d]

# # 绘制散点图
# plt.scatter(x, y)
# plt.show()
o3d.visualization.draw_geometries(pcds)
# 创建可视化窗口
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

# vis.add_geometry(origin)

# colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1]]
# for i in range(0, len(pcds), 2):
#     print(i)
#     vis.add_geometry(pcds[i].paint_uniform_color(colors[int(i/2)]))
#     vis.add_geometry(pcds[i+1].paint_uniform_color(colors[int(i/2)]))

# vis.run()
# vis.destroy_window()
    

