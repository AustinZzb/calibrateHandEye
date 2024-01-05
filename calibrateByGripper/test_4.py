'''
Author: Austin 1158680540@qq.com
Date: 2023-12-25 11:02:09
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2024-01-04 17:26:45
FilePath: \calibrateHandEye\calibrateByGripper\test_4.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import numpy as np

def plane_equation_through_AB(A, B):
    # A, B 分别为点 A 和点 B 的坐标
    x1, y1, z1 = A
    x2, y2, z2 = B
    
    # 计算向量 AB
    AB = np.array([x2 - x1, y2 - y1, z2 - z1])
    
    # 过 AB 中点的坐标，可以取中点坐标
    midpoint = np.array([(x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2])
    
    # 构建平面方程
    x0, y0, z0 = midpoint
    
    equation = f"{AB[0]}(x - {x0}) + {AB[1]}(y - {y0}) + {AB[2]}(z - {z0}) = 0"
    
    return equation

# 例子：假设点A为(1, 2, 3)，点B为(4, 5, 6)
point_A = (-0.272, 0, 37.415602828667474)
point_B = (14.552, 0, 40.97912273121355)

plane_equation = plane_equation_through_AB(point_A, point_B)
print("垂直于AB且过AB中点的平面方程为:", plane_equation)
