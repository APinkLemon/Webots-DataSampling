# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年03月11日
"""

from easydict import EasyDict as edict

__C = edict()
cfg = __C

__C.param = edict()
__C.param.savePeriod = 50
__C.param.basePath = "dataSet1/"
__C.param.sample = True
__C.param.velocity = 50

__C.robot = edict()
__C.robot.time = 50.0

__C.lidar = edict()
__C.lidar.isEnable = True
__C.lidar.samplingPeriod = 50
__C.lidar.frequency = 5.0

__C.gps = edict()
__C.gps.isEnable = True
__C.gps.samplingPeriod = 50

__C.camera = edict()
__C.camera.isEnable = True
__C.camera.samplingPeriod = 50

__C.sick = edict()
__C.sick.isEnable = True
__C.sick.samplingPeriod = 50

__C.pid = edict()
__C.pid.isEnable = True
__C.pid.KP = 0.25
__C.pid.KI = 0.006
__C.pid.KD = 2
