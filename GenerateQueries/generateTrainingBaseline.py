# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年03月26日
"""

import sys
import pickle
import random
import numpy as np
import pandas as pd
from config import cfg
from sklearn.neighbors import KDTree


deltaX = 50
deltaY = 50
p1 = [24.273913896704368, 104.54332989046137]
p2 = [27.632015077993096, -45.41815255413447]
p = [p1, p2]


def checkInTestSet(x_pos, y_pos, test_points, x_width, y_width):
    in_test_set = False
    for point in test_points:
        if point[0] - x_width < x_pos < point[0] + x_width and point[1] - y_width < y_pos < point[1] + y_width:
            in_test_set = True
            break
    return in_test_set


def constructQueryDict(df_centroids, filename):
    tree = KDTree(df_centroids[['northing', 'easting']])
    ind_nn = tree.query_radius(df_centroids[['northing', 'easting']], r=10)
    ind_r = tree.query_radius(df_centroids[['northing', 'easting']], r=50)
    queries = {}
    for i in range(len(ind_nn)):
        query = df_centroids.iloc[i]["timestamp"]
        positives = np.setdiff1d(ind_nn[i], [i]).tolist()
        negatives = np.setdiff1d(df_centroids.index.values.tolist(), ind_r[i]).tolist()
        random.shuffle(positives)
        random.shuffle(negatives)
        queries[i] = {"query": query, "positives": positives, "negatives": negatives}

    with open(filename, 'wb') as handle:
        # print(queries)
        pickle.dump(queries, handle, protocol=pickle.HIGHEST_PROTOCOL)

    print("Construct Training Baseline Done: " + filename + "!")


df_train = pd.DataFrame(columns=['timestamp', 'northing', 'easting'])
df_test = pd.DataFrame(columns=['timestamp', 'northing', 'easting'])
df_locations = pd.read_csv("../dataTrain" + str(cfg.param.trainNum) + ".csv")
df_locations['timestamp'] = "dataTrain" + str(cfg.param.trainNum) + "/" + df_locations['timestamp'].astype(str) + ".npy"

for index, row in df_locations.iterrows():
    if checkInTestSet(row['northing'], row['easting'], p, deltaX, deltaY):
        df_test = df_test.append(row, ignore_index=True)
    else:
        df_train = df_train.append(row, ignore_index=True)

print("Number of training submaps: " + str(len(df_train['timestamp'])))
print("Number of non-disjoint test submaps: " + str(len(df_test['timestamp'])))
constructQueryDict(df_train, "train_queries_baseline_" + str(cfg.param.trainNum) + ".pickle")
constructQueryDict(df_test, "test_queries_baseline_" + str(cfg.param.trainNum) + ".pickle")
