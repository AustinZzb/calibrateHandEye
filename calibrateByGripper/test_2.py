import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button, RadioButtons

# 定义球体的半径
radius = 2

# 定义起点和终点的初始坐标
A0 = np.array([1, 2, 3])
B0 = np.array([4, 5, 6])

# 定义起点和终点的可调节坐标
A = A0.copy()
B = B0.copy()

# 计算球体的圆心坐标
center = (A + B) / 2

# 创建图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制球体
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = center[0] + radius*np.cos(u)*np.sin(v)
y = center[1] + radius*np.sin(u)*np.sin(v)
z = center[2] + radius*np.cos(v)
sphere = ax.plot_wireframe(x, y, z, color='b', alpha=0.2)

# 绘制线段
line = ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], color='k')[0]

# 定义鼠标拖动回调函数
def on_move(event):
    if event.inaxes != ax:
        return
    if event.button == 1:
        A[:] = A0 + np.array([event.xdata, event.ydata, 0])
    elif event.button == 3:
        B[:] = B0 + np.array([event.xdata, event.ydata, 0])
    update()

# 更新图形
def update():
    # 计算球体的圆心坐标
    center[:] = (A + B) / 2

    # 更新球体
    x = center[0] + radius*np.cos(u)*np.sin(v)
    y = center[1] + radius*np.sin(u)*np.sin(v)
    z = center[2] + radius*np.cos(v)
    sphere.set_data(x, y)
    sphere.set_3d_properties(z)

    # 更新线段
    line.set_xdata([A[0], B[0]])
    line.set_ydata([A[1], B[1]])
    line.set_3d_properties([A[2], B[2]])

    # 更新坐标轴范围
    ax.set_xlim(center[0]-radius-1, center[0]+radius+1)
    ax.set_ylim(center[1]-radius-1, center[1]+radius+1)
    ax.set_zlim(center[2]-radius-1, center[2]+radius+1)

    # 重新绘制图形
    fig.canvas.draw_idle()

# 绑定鼠标拖动事件
fig.canvas.mpl_connect('motion_notify_event', on_move)

plt.show()
