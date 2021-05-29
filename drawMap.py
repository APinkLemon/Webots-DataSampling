# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年05月29日
"""

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


gs = GridSpec(3, 2)
fig = plt.figure()
table = pd.read_csv("dataTrain3.csv")
print(table)
x = table["northing"].tolist()
y = table["easting"].tolist()
ax1 = fig.add_subplot(gs[0:3, 0:2])
ax1.scatter(x, y, marker='*', color='b')
plt.show()
