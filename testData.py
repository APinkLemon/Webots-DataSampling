# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年05月08日
"""

import pandas as pd
import dataProcess


b = dataProcess.getFilePathList("dataTrain2")
for i in range(len(b)):
    # print(b[i][11:-4])
    b[i] = b[i][11:-4]

print(b)
print("$"*100)

data = pd.read_csv("dataTrain2.csv")
for j in range(len(data)):
    if j == 0:
        continue
    # print(j)
    # print(data.keys()[0])
    a = data[data.keys()[0]][j]
    a = str(a)
    # if j == 1:
    # print(a)
    # print(b[1])
    # print(type(a))
    # print(type(b[1]))
    # print(a in b)
    if a not in b:
        print(a)
    # import sys
    # sys.exit(989)
    # print(data[j][0])

