'''
Author: Austin 1158680540@qq.com
Date: 2023-12-19 13:11:17
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-12-20 14:07:44
FilePath: \calibrateHandEye\scikit_PSO.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import numpy as np
from sko.PSO import PSO
from sko.tools import set_run_mode
import calibrateByPad.PSO_function as pso_function_pad
import calibrateByTest.PSO_function as pso_function_test
import calibrateByGripper.PSO_function as pso_function_gripper

pso_function = pso_function_test

def demo_func(x):
    x1, x2, x3 = x
    return -20 * np.exp(-0.2 * np.sqrt(0.5 * (x1 ** 2 + x2 ** 2))) - np.exp(
        0.5 * (np.cos(2 * np.pi * x1) + np.cos(2 * np.pi * x2))) + 20 + np.e + x3


constraint_ueq = (
    lambda x: (x[0] - 1) ** 2 + (x[1] - 0) ** 2 + x[2] - 0.5 ** 2
    ,
)

max_iter = 100
pso = PSO(func=pso_function.pso_function, n_dim=7, pop=100, max_iter=max_iter, lb=[-2, -2, -2, -0.8, -100, 100, 1125], ub=[2, 2, 2, 0.8, 100, 300, 1200]
          , constraint_ueq=[pso_function.constraint_ueq_1, pso_function.constraint_ueq_2])
set_run_mode(pso_function.pso_function, 'vectorization')

pso.record_mode = True
pso.run()
print(pow(pso.gbest_x[0], 2) + pow(pso.gbest_x[1], 2) + pow(pso.gbest_x[2], 2) + pow(pso.gbest_x[3], 2))
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
