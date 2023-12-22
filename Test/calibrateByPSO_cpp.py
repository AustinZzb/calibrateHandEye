import ctypes
import subprocess
import numpy as np

def getDLL():
    # 要执行的命令
    command = '''g++ -shared -o C:\\Users\\austin.zhang\\Code\\VSCode\\calibrateHandEye\\pyFile\\PSO.dll C:\\Users\\austin.zhang\\Code\\VS\\PSO_calibrate\\PSO.cpp -IC:\\Users\\austin.zhang\\DEV\\PCL1_13_0\\3rdParty\\Eigen\\eigen3'''

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
my_lib.main()

