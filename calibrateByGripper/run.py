'''
Author: Austin 1158680540@qq.com
Date: 2023-12-20 15:14:53
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-12-22 15:42:36
FilePath: \calibrateHandEye\calibrateByGripper\run.py
Description: 执行文件入口
'''
import numpy as np
from sko.PSO import PSO
from sko.tools import set_run_mode
import DataLoader
import PSO_function
import threading

def pso_run(args):   
    pso = PSO(**args)
    # set_run_mode(args['func'], 'vectorization')
    set_run_mode(args['func'], 'cached')
    # pso.record_mode = True
    # print(pso.record_value['X']) # 打印记录的粒子位置信息
    # print(pso.record_value['Y']) # 打印记录的函数值
    pso.run()

    print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)
        
    del pso
    # return pso.gbest_x, pso.gbest_y


dataLoader_1 = DataLoader.handleData_1(
    R=17.5, C={'x': 4.7, 'y': 0, 'z': 20}, D={'x': 0, 'y': 0, 'z': 20})

PSO_func_1 = PSO_function.PSO_function_1()
PSO_func_1.setDataLoader(dataLoader_1)

args={'func': PSO_func_1.pso_function, 'n_dim': 3, 'max_iter': 500,
      'pop': 1000, 'lb': [-20, 0, 0], 'ub': [20, 50, 20], 'verbose': False,
      'constraint_ueq': PSO_func_1.getConstraintList()}

sphericalCenterList = []

for i in range(10):
    t = threading.Thread(target=pso_run(args))
    t.start()

