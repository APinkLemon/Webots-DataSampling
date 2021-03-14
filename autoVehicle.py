# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年03月05日
"""

import time
from controller import Lidar, Keyboard, LidarPoint, GPS
from vehicle import Car
from config import cfg


basePath = cfg.param.basePath


def savePointCloudTxt(gpsInfo, pointCloud, filename):
    fw = open(filename, 'w')
    fw.write(str(gpsInfo))
    fw.write("\n")
    for i in range(len(pointCloud)):
        fw.write(str(pointCloud[i].x) + " " + str(pointCloud[i].y) + " " + str(pointCloud[i].z) +
                 " " + str(pointCloud[i].layer_id) + " " + str(pointCloud[i].time))
        fw.write("\n")
    fw.close()
    print(filename + "Saved!")


class OnHandVehicle(Car):
    def __init__(self):
        Car.__init__(self)
        self.getType()
        self.getEngineType()
        self.lidar = self.getDevice("lidar")
        self.lidar.enable(5000)
        self.lidar.enablePointCloud()
        print(self.lidar.isPointCloudEnabled())
        self.keyboard.enable(10)
        self.speed = 0
        print("^"*10)

    def run(self):
        print("Hello")
        self.speed = 0
        print(type(self.lidar))
        A = self.lidar.getNumberOfPoints()
        print(A)
        while self.step() != -1:
            key = self.keyboard.getKey()

            if key == Keyboard.LEFT:
                self.setSteeringAngle(-0.05)
            elif key == Keyboard.RIGHT:
                self.setSteeringAngle(0.05)
            elif key == Keyboard.UP:
                self.speed += 5
                self.setCruisingSpeed(self.speed)
            elif key == Keyboard.DOWN:
                self.speed -= 5
                self.setCruisingSpeed(self.speed)
            elif key == ord('B'):
                B = self.lidar.getPointCloud(data_type='list')
                tmp = []
                print('B is pressed')
                print(len(B))


class AutoVehicle(Car):
    def __init__(self):
        Car.__init__(self)
        self.cameraChannel = 3
        self.controlTime = cfg.robot.time
        self.basicTime = self.getBasicTimeStep()
        print("#"*40)
        print("This is auto-Vehicle!")
        print("Here are my Info: ")
        print("My Synchronization: ", self.getSynchronization())
        print("My Basic Time Step: ", self.getBasicTimeStep())
        print("My Control Time Step: ", self.controlTime)
        print("#" * 40)

        print("Here are my Device Info: ")
        print("#" * 40)
        print("Lidar Enabled: ", cfg.lidar.isEnable)
        if cfg.lidar.isEnable:
            self.lidar = self.getDevice("lidar")
            self.lidar.enable(cfg.lidar.samplingPeriod)
            self.lidar.enablePointCloud()
            self.lidar.setFrequency(cfg.lidar.frequency)
            print("Sampling Period: ", self.lidar.getSamplingPeriod())
            print("Rotation Frequency: ", self.lidar.getFrequency())
        print("#" * 40)

        print("GPS Enabled: ", cfg.gps.isEnable)
        if cfg.gps.isEnable:
            self.gps = self.getDevice("gps")
            self.gps.enable(cfg.gps.samplingPeriod)
            print("Sampling Period: ", self.gps.getSamplingPeriod())
        print("#" * 40)

        print("Camera Enabled: ", cfg.camera.isEnable)
        if cfg.camera.isEnable:
            self.camera = self.getDevice("camera")
            self.camera.enable(cfg.camera.samplingPeriod)
            self.cameraImgWidth = self.camera.getWidth()
            self.cameraImgHeight = self.camera.getHeight()
            self.cameraImgFov = self.camera.getFov()
            print("Sampling Period: ", self.camera.getSamplingPeriod())
            print("Image Width: ", self.cameraImgWidth)
            print("Image Height: ", self.cameraImgHeight)
        print("#" * 40)

        print("Sick Enabled: ", cfg.sick.isEnable)
        if cfg.sick.isEnable:
            self.sick = self.getDevice("Sick LMS 291")
            self.sick.enable(cfg.sick.samplingPeriod)
            print("Sampling Period: ", self.sick.getSamplingPeriod())
        print("#" * 40)

        self.setHazardFlashers(True)
        self.setDippedBeams(True)
        self.setAntifogLights(True)
        self.setWiperMode(False)
        self.setCruisingSpeed(10)

    def colorDiff(self, color1, color2):
        diff = 0
        for i in range(self.cameraChannel):
            diff += abs(color1[i] - color2[i])
        return diff

    def processCameraImage(self, imageInfo):
        REF = [203, 187, 95]
        xDiffSum = 0
        yellowPixels = 0

        for i in range(self.cameraImgWidth):
            for j in range(self.cameraImgHeight):
                if self.colorDiff(imageInfo[i][j], REF) < 30:
                    xDiffSum += i % self.cameraImgWidth
                    yellowPixels += 1

        print(yellowPixels)
        if yellowPixels == 0:
            return 9999
        return (xDiffSum / yellowPixels / self.cameraImgWidth - 0.5) * self.cameraImgFov

    def run(self):
        i = 1
        while self.step() != -1:
            self.setCruisingSpeed(10)
            if i % int(self.controlTime / self.basicTime) == 0:
                if cfg.camera.isEnable:
                    camera_data = self.camera.getImageArray()
                    self.processCameraImage(camera_data)
                if cfg.sick.isEnable:
                    sick_data = self.sick.getRangeImage()

            if i % (cfg.param.savePeriod * int(self.controlTime / self.basicTime)) == 0 and cfg.param.sample == 1:
                if cfg.gps.isEnable == 1 and cfg.lidar.isEnable == 1:
                    path = basePath + str(int(time.time())) + ".txt"
                    gpsInfo = self.gps.getValues()
                    lidarInfo = self.lidar.getPointCloud(data_type='list')
                    savePointCloudTxt(gpsInfo, lidarInfo, path)
            i += 1


BmwX5 = AutoVehicle()
BmwX5.run()
