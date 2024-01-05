'''
Author: Austin 1158680540@qq.com
Date: 2023-12-15 10:25:42
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-03 10:12:21
FilePath: /calibrateHandEye/test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import open3d as o3d
import numpy as np
import open3dUtil as o3dUtil

def padLine():
    with open('Data/Line/1.1.txt', 'r') as f:
        lines = f.readlines()
        
        strPoints = [line.strip().split() if " " in line else line.strip().split('/t') for line in lines]

        points = []
        for strPoint in strPoints:
            if np.double(strPoint[1]) != -10000:
                print(strPoint[1])
                points.append([np.double(strPoint[0]), 0, np.double(strPoint[1])])
        # print(points)
        pcd = o3dUtil.createPCD(points)
        print(pcd)
        o3dUtil.showPointCloud(pcd)

# 从txt文件中读取点云数据
# pcd = o3d.io.read_point_cloud("./Data/LMI_gripper_calibrate/replay_128136_2023-12-14 (2).txt", format='xyz')

def getTxtToPCD():
    source = o3d.geometry.PointCloud()
    m1 = np.loadtxt('Data/LMI_gripper_calibrate/Line/replay_128136_2023-12-14 (4).txt', delimiter=",")
    source.points = o3d.utility.Vector3dVector(m1)

    print(source)
    # 可视化点云
    o3d.visualization.draw_geometries([source])


def getTxtToPCD_Point():
    # 读取点云数据
    m1 = np.loadtxt('Data/LMI_gripper_calibrate/Line/replay_128136_2023-12-14 (4).txt', delimiter=",")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(m1)

 
    point_cloud2 = pcd.voxel_down_sample(voxel_size=0.01)  # voxel_down_sample 把点云分配在三维的网格中取平均值
    
    print("   按住 [shift + right click] 选择点")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(point_cloud2)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    pick_p = vis.get_picked_points()
    print(pick_p)  # 会打印出选择的点的序号


    
getTxtToPCD_Point()
# getTxtToPCD()