'''
Author: Austin 1158680540@qq.com
Date: 2023-11-15 15:52:45
LastEditors: Austin 1158680540@qq.com
LastEditTime: 2023-11-15 16:01:11
FilePath: \calibrateHandEye\pyFile\calibrateByPSO_cpp copy.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import ctypes
import subprocess
import numpy as np

def getDLL():
    # 要执行的命令
    command = '''g++ -shared -o PSO.dll C:\\Users\\austin.zhang\\Code\\VS\\PSO_calibrate\\PSO.cpp -IC:\\Users\\austin.zhang\\DEV\\PCL1_13_0\\3rdParty\\Eigen\\eigen3'''


    # 创建一个subprocess.PIPE对象，用于隐藏窗口
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW

    # 执行命令，并隐藏窗口
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, startupinfo=startupinfo)

    # 获取命令输出
    output, error = process.communicate()

    # 输出命令输出和错误信息
    print(output.decode('gbk'))
    print(error.decode('gbk'))

getDLL()

# 加载动态链接库文件
my_lib = ctypes.cdll.LoadLibrary('pyFile\\PSO.dll')

# 调用函数
print(my_lib.PSO(1, 2))

