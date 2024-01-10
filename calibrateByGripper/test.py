import sys
sys.path.append('./')
import calibrateUtil_scipy as util
import calibrateUtil as myUtil
import numpy as np


funList = []
for i in range(10):
    funList.append(lambda a: fun(a, i))
def fun(a, x):
    print(a, x)
print(funList)
for i in range(10):
    funList[i]("sssss")


