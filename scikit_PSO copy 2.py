'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 13:11:17
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-12-20 14:53:43
FilePath: \calibrateHandEye\scikit_PSO.py
Description: 函数参数https://zhuanlan.zhihu.com/p/301281157
'''
import numpy as np
from sko.PSO import PSO
from sko.tools import set_run_mode


def demo_func(x):
    x1 = x
    return -(-x1**2 + 1)


# constraint_ueq = (
#     lambda x: (x[0] - 1) ** 2 + (x[1] - 0) ** 2 + x[2] - 0.5 ** 2
#     ,
# )

max_iter = 50
pso = PSO(func=demo_func, n_dim=1, pop=40, max_iter=max_iter, lb=[-2], ub=[2])
set_run_mode(demo_func, 'vectorization')

pso.record_mode = True
pso.run()
print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)

# # %% Now Plot the animation
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# record_value = pso.record_value
# X_list, V_list = record_value['X'], record_value['V']

# fig, ax = plt.subplots(1, 1)
# ax.set_title('title', loc='center')
# line = ax.plot([], [], 'b.')

# X_grid, Y_grid = np.meshgrid(np.linspace(-2.0, 2.0, 40), np.linspace(-2.0, 2.0, 40))
# Z_grid = demo_func((X_grid, Y_grid))
# ax.contour(X_grid, Y_grid, Z_grid, 30)

# ax.set_xlim(-2, 2)
# ax.set_ylim(-2, 2)

# t = np.linspace(0, 2 * np.pi, 40)
# ax.plot(0.5 * np.cos(t) + 1, 0.5 * np.sin(t), color='r')

# plt.ion()
# p = plt.show()


# def update_scatter(frame):
#     i, j = frame // 10, frame % 10
#     ax.set_title('iter = ' + str(i))
#     X_tmp = X_list[i] + V_list[i] * j / 10.0
#     plt.setp(line, 'xdata', X_tmp[:, 0], 'ydata', X_tmp[:, 1])
#     return line


# ani = FuncAnimation(fig, update_scatter, blit=True, interval=25, frames=max_iter * 10)
# plt.show()

# ani.save('pso.gif', writer='pillow')
# # %%