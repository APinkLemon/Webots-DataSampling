# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年03月14日
"""

import open3d as o3d
import numpy as np


def storeDataToNumpy(path, render=True):
    j = 0
    data = []
    for line in open(path):
        single_data = line.split(' ')[0:-2]
        if j == 0:
            j += 1
            continue
        for i in range(len(single_data)):
            single_data[i] = float(single_data[i])
        if abs(single_data[0]) > 10000 or abs(single_data[1]) > 10000 or abs(single_data[2]) > 10000:
            continue
        if render:
            print(single_data)
        data.append(single_data)

    data_numpy = np.array(data)
    savePath = path[0:7]+"Npy"+path[7:-4]+".npy"
    np.save(savePath, data_numpy)
    print(data_numpy.shape)
    print("Successfully Saved!")
    return savePath


def storeDataToPcd(path):
    data_numpy = np.load(path).reshape(-1, 3)
    print(data_numpy.shape)
    exp = o3d.geometry.PointCloud()
    exp.points = o3d.utility.Vector3dVector(data_numpy)
    savePath = path[0:7]+"Pcd"+path[10:-4]+".pcd"
    o3d.io.write_point_cloud(savePath, exp)
    print("Successfully Saved!")
    return savePath


def visionPointCloud(pointCloud):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pointCloud)
    vis.update_geometry(pointCloud)
    vis.poll_events()
    vis.update_renderer()
    vis.run()


def visionPointCloudPcd(path):
    exp = o3d.io.read_point_cloud(path)
    visionPointCloud(exp)


mypath = "dataSet1/1615479189.txt"
print(storeDataToPcd(storeDataToNumpy(mypath)))
# visionPointCloudPcd(mypath)
