'''
Author: Austin 58591300+Justin-zzb@users.noreply.github.com
Date: 2023-12-22 21:57:00
LastEditors: Austin 58591300+Justin-zzb@users.noreply.github.com
LastEditTime: 2023-12-22 22:06:08
FilePath: \calibrateHandEye-1\calibrateByGripper\test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_sphere_and_circle(center, radius, points):
    # 生成球体表面上的点
    phi, theta = np.mgrid[0.0:2.0*np.pi:100j, 0.0:np.pi:50j]
    x_sphere = center[0] + radius * np.sin(theta) * np.cos(phi)
    y_sphere = center[1] + radius * np.sin(theta) * np.sin(phi)
    z_sphere = center[2] + radius * np.cos(theta)

    # 绘制球体
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x_sphere, y_sphere, z_sphere, color='b', alpha=0.3)

    # 绘制前两个点
    x_points, y_points, z_points = zip(*points[:2])
    ax.scatter(x_points, y_points, z_points, color='r', marker='o')

    # 计算并绘制以第三个点为圆心的圆
    circle_center = points[2]
    circle_radius = np.linalg.norm(np.array(points[0]) - np.array(points[2]))  # 任意选取一个点到第三个点的距离作为圆的半径
    phi_circle = np.linspace(0, 2*np.pi, 100)
    x_circle = circle_center[0] + circle_radius * np.cos(phi_circle)
    y_circle = circle_center[1] + circle_radius * np.sin(phi_circle)
    z_circle = circle_center[2] * np.ones_like(phi_circle)  # 圆所在的平面

    # 绘制圆
    ax.plot(x_circle, y_circle, z_circle, color='g', label='Circle')

    # 设置图形属性
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Sphere with Circle')

    # 添加图例
    ax.legend()

    # 显示图形
    plt.show()

# 示例
center = (-6.75252788e-17, 9.05510933e-01, 5.10067043e+01)
radius = 20
points = [(-5, 0, 50), (5, 0, 50), (3.77064398e-15, -1.34567978e+01, 6.39253220e+01)]

plot_sphere_and_circle(center, radius, points)
