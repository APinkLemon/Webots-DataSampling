# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年04月04日
"""


import pandas as pd
from config import cfg
from dataProcess import getFilePathList


pathList = getFilePathList(cfg.param.basePath)
posX = []
posY = []
filename = []
for i in pathList:
    print(i[-14:-4])
    filename.append(i[-14:-4])
    for line in open(i):
        single_data = line[1:-2].split(', ')
        single_data = list(map(float, single_data))
        posX.append(single_data[0])
        posY.append(single_data[2])
        break
dataDict = {"timestamp": filename, "northing": posX, "easting": posY}
data = pd.DataFrame(dataDict)
print(data)
data.to_csv(
    './dataTrain' + str(cfg.param.trainNum) + '.csv',
    index=False,  # 不保存行索引
    header=True,  # 保存列索引
)
