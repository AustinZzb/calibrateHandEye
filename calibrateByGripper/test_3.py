import numpy as np
import open3d as o3d

# 生成点云数据
points = np.array([[ 8.15010710e-15,-1.70169906e+01, 4.07575960e+01] ,
                   [ 3.77064398e-15,-1.34567978e+01, 6.39253220e+01],
                   [-3.70242133e-15 ,-1.70035857e+01 , 5.92670424e+01  ] ,
                   [ 1.17432263e-14  ,9.23464617e+00 , 6.70212018e+01] ,
                   [-2.93732114e-15 , 3.18413944e+00 , 3.08986583e+01] ,
                   [-2.52884103e-16 ,-1.89448028e+01,  5.40117884e+01] ,
                   [-2.04541982e-15 ,-7.65339697e+00 , 6.77883533e+01] ,
                   [-4.81084544e-15, -1.85333034e+01 , 5.56139705e+01] ,
                   [ 4.90588579e-15 , 1.88898290e+01 , 4.57368601e+01  ],
                   [6.88846381e-15 ,1.89646190e+01 ,5.39170429e+01 ]] )

# 定义点A和点B的坐标
A = np.array([-5, 0, 50])
B = np.array([5, 0, 50])

# 计算向量AB的坐标
AB = B - A

# 计算点C的坐标
C = (A + B) / 2

# 计算平面方程的系数
ABC = np.array([AB[0], AB[1], AB[2]])
D = -np.dot(C, ABC)

# 输出平面方程
print(f"平面方程为 {ABC[0]}x + {ABC[1]}y + {ABC[2]}z + {D} = 0")

# 计算每个点到平面的距离
distances = np.abs(np.dot(points, ABC) + D) / np.linalg.norm(ABC)

# 输出每个点到平面的距离
for i in range(points.shape[0]):
    print(f"点{i+1}到平面的距离为 {distances[i]}")

# 将点云数据转换为Open3D的点云对象
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# 计算点云数据的法向量
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# 将距离作为颜色信息，添加到点云对象中
colors = np.zeros((10, 3))
colors[:, 0] = distances / np.max(distances)
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd])


