'''
Author: Austin 1158680540@qq.com
Date: 2023-12-20 15:14:53
LastEditors: Austin 58591300+Justin-zzb@users.noreply.github.com
LastEditTime: 2023-12-22 19:59:45
FilePath: \calibrateHandEye\calibrateByGripper\run.py
Description: 执行文件入口
'''
import numpy as np
from sko.PSO import PSO
from sko.tools import set_run_mode
import DataLoader
import PSO_function
import threading
from math import sqrt

def getParam():
    Param = {
        'R': 20,
        'C': {'x': -5, 'y': 0, 'z': 50},
        'D': {'x': 5, 'y': 0, 'z': 50}
    }

    return Param

def getDE(Points):
    list = Points.tolist()
    D = getParam()['D']

    print('AD', sqrt(pow(list[0]-D['x'], 2) + pow(list[1]-D['y'], 2) + pow(list[2]-D['z'], 2)))
    print('DE', sqrt(pow(list[3]-D['x'], 2) + pow(list[4]-D['y'], 2) + pow(list[5]-D['z'], 2)))
    print('AE', sqrt(pow(list[3]-list[0], 2) + pow(list[4]-list[1], 2) + pow(list[5]-list[2], 2)))
    

def pso_run(args):   
    pso = PSO(**args)
    # set_run_mode(args['func'], 'vectorization')
    set_run_mode(args['func'], 'cached')
    # pso.record_mode = True
    # print(pso.record_value['X']) # 打印记录的粒子位置信息
    # print(pso.record_value['Y']) # 打印记录的函数值
    pso.run()

    print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)
    getDE(pso.gbest_x)


dataLoader_1 = DataLoader.handleData_1(**getParam())

PSO_func_1 = PSO_function.PSO_function_1()
PSO_func_1.setDataLoader(dataLoader_1)

args={'func': PSO_func_1.pso_function, 'n_dim': 6, 'max_iter': 500,
      'pop': 1000, 'lb': [-20, -20, 20, -20, -20, 20], 'ub': [20, 20, 70, 20, 20, 70], 'verbose': False,
      'constraint_ueq': PSO_func_1.getConstraintList()}

sphericalCenterList = []

for i in range(10):
    t = threading.Thread(target=pso_run(args))
    t.start()

