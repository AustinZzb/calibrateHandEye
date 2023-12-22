'''
@ name: Austin
@ description: 一些open3d常用到的函数
@ Date: 2023-10-13 14:02:49
@ LastEditDate: 
'''

from os import path
import random
import numpy as np
import open3d as o3d
from zmq import NULL
from sklearn.preprocessing import MinMaxScaler

def readOrWrite(fileName, IOtype = 'read', fileType='.pcd', pcd = NULL, path = './pcdFile/'):
    '''读写PCD

    Args:
        fileName (_type_): _description_
        IOtype (_type_): _description_
        fileType (str, optional): _description_. Defaults to '.pcd'.
        pcd (_type_, optional): _description_. Defaults to NULL.
        path (str, optional): _description_. Defaults to './pcdFile/'

    Returns:
        pcd: _description_
    '''
    if IOtype == "read":
        return o3d.io.read_point_cloud(path+fileName+fileType)
    else:
        o3d.io.write_point_cloud(path+fileName+fileType, pcd, True)


def paintForDifferent(pcdList, isFamily=False):
    '''传入一个pcd列表，对其中各pcd上色\n
    并直接调用showPointCloud可视化\n
    isFamily=True时，默认len(pcdList)=2，且前者是后者的子集

    Args:
        pcdList (list): _description_\n
        isFamily (bool): 是否是同一pcd文件，前者为子集，后者为原pcd
    '''
    if isFamily:
        colors = [[1, 1, 1] if i in pcdList[0].points else [0, 0, 0] for i in pcdList[1].points]
        pcdList[1].colors = o3d.utility.Vector3dVector(colors)
    else:
        for i in range(0, len(pcdList)):
            pcdList[i].paint_uniform_color([random.random(), random.random(), random.random()])

    showPointCloud(pcdList)

def showPointCloud(pcd):
    '''传入单个pcd或列表进行可视化

    Args:
        pcd (_type_): _description_
    '''
    if list == type(pcd):
        o3d.visualization.draw_geometries(pcd)
    else: 
        o3d.visualization.draw_geometries([pcd])
        

def createPCD(points):
    '''根据传入点的点集创建pcd

    Args:
        points (list): _description_

    Returns:
        pcd: _description_
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))

    return pcd

def downVoxel(pcd, voxel_size = 0.01):    
    '''下采样

    Args:
        pcd (_type_): _description_
        voxel_size (球半径, optional): _description_. Defaults to 0.01.
    '''
    down_long_pcd = pcd.voxel_down_sample(voxel_size)
    return down_long_pcd

def getOrigin(pcd, radius = 0.5, color = [1, 0, 0]):
    '''在pcd原点处放置一个半径为radius，颜色为color的球

    Args:
        pcd (_type_): _description_
        radius (float, optional): 原点球半径. Defaults to 0.5.
        color (list, optional): 原点球颜色. Defaults to [1, 0, 0].
    '''
    # 指定球体参数
    center = [0, 0, 0]  # 球心坐标
    color = color  # 球体颜色

    # 创建球体
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)

    # 将球体放置在指定坐标
    sphere.translate(center)

    # 将球体转换为点云类型
    sphere_pc = o3d.geometry.PointCloud()
    sphere_pc.points = sphere.vertices
    sphere_pc.colors = sphere.vertex_colors

    # 将球体添加到点云中
    pcd += sphere_pc

    # 可视化点云
    o3d.visualization.draw_geometries([pcd])
