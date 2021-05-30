import sys
import time
from controller import Lidar, Keyboard, LidarPoint, GPS
from vehicle import Car
from config import cfg


unKnown = 9999


def savePointCloudTxt(gpsInfo, pointCloud, basePath=cfg.param.basePath):
    filename = basePath + str(int(time.time())) + ".txt"
    fw = open(filename, 'w')
    fw.write(str(gpsInfo))
    fw.write("\n")
    for i in range(len(pointCloud)):
        fw.write(str(pointCloud[i].x) + " " + str(pointCloud[i].y) + " " + str(pointCloud[i].z) +
                 " " + str(pointCloud[i].layer_id) + " " + str(pointCloud[i].time))
        fw.write("\n")
    fw.close()
    print(filename + "Saved!")


class AutoVehicle(Car):
    def __init__(self):
        Car.__init__(self)
        self.saveStep = 0
        self.steeringAngle = 0.0
        self.velocity = cfg.param.velocity
        self.controlTime = cfg.robot.time
        self.basicTime = self.getBasicTimeStep()
        print("#" * 40)
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
            self.firstCall = True
            self.filterSize = 3
            self.cameraChannel = 3
            self.oldCameraValue = [0, 0, 0]
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
            self.HALFAREA = 20
            self.sick = self.getDevice("Sick LMS 291")
            self.sick.enable(cfg.sick.samplingPeriod)
            self.sickWidth = self.sick.getHorizontalResolution()
            self.sickRange = self.sick.getMaxRange()
            self.sickFov = self.sick.getFov()
            print("Sampling Period: ", self.sick.getSamplingPeriod())
            print("Sick Width: ", self.sickWidth)
            print("Sick Range: ", self.sickRange)
            print("Sick Fov: ", self.sickFov)
        print("#" * 40)

        print("PID Enabled: ", cfg.pid.isEnable)
        if cfg.pid.isEnable:
            self.needResetPID = False
            self.oldPIDValue = 0.0
            self.integral = 0.0
            print("KP: ", cfg.pid.KP)
            print("KI: ", cfg.pid.KI)
            print("KD: ", cfg.pid.KD)
        print("#" * 40)

        self.setHazardFlashers(True)
        self.setDippedBeams(True)
        self.setAntifogLights(True)
        self.setWiperMode(False)
        self.setCruisingSpeed(self.velocity)

    def processSickInfo(self, sickInfo):
        sumX = 0
        obstacleDist = 0
        collisionCount = 0
        for i in range(self.HALFAREA * 2):
            key = int(i + self.sickWidth / 2 - self.HALFAREA)
            tmp = sickInfo[key]
            if tmp < 20.0:
                sumX += key
                collisionCount += 1
                obstacleDist += tmp
        if collisionCount == 0:
            return unKnown, obstacleDist
        obstacleDist = obstacleDist / collisionCount
        obstacleAngle = (sumX / collisionCount / self.sickWidth - 0.5) * self.sickFov
        return obstacleAngle, obstacleDist

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

        if yellowPixels == 0:
            return unKnown
        return (xDiffSum / yellowPixels / self.cameraImgWidth - 0.5) * self.cameraImgFov

    def filterAngle(self, newValue):
        if self.firstCall or newValue == unKnown:
            self.firstCall = False
        else:
            for i in range(self.filterSize - 1):
                self.oldCameraValue[i] = self.oldCameraValue[i + 1]
        if newValue == unKnown:
            return unKnown
        else:
            self.oldCameraValue[self.filterSize - 1] = newValue
            filterSum = 0.0
            for i in range(self.filterSize):
                filterSum += self.oldCameraValue[i]
            return filterSum / self.filterSize

    def applyPID(self, yellowLineAngle):
        if self.needResetPID:
            self.oldPIDValue = yellowLineAngle
            self.integral = 0
            self.needResetPID = False
        if self.oldPIDValue * yellowLineAngle < 0:
            self.integral = 0
        if abs(self.integral) < 30:
            self.integral += yellowLineAngle
        diff = yellowLineAngle - self.oldPIDValue
        self.oldPIDValue = yellowLineAngle
        return cfg.pid.KP * yellowLineAngle + cfg.pid.KI * self.integral + cfg.pid.KD * diff

    def run(self):
        i = 1
        while self.step() != -1:
            if i % int(self.controlTime / self.basicTime) == 0:
                if cfg.camera.isEnable:
                    cameraData = self.camera.getImageArray()
                    yellowLineAngle = self.filterAngle(self.processCameraImage(cameraData))
                    if cfg.sick.isEnable:
                        sickData = self.sick.getRangeImage()
                        obstacleAngle, obstacleDist = self.processSickInfo(sickData)
                        if obstacleAngle != unKnown:
                            self.setBrakeIntensity(0.0)
                            obstacleSteering = self.steeringAngle
                            if 0.0 < obstacleAngle < 0.4:
                                obstacleSteering = self.steeringAngle + (obstacleAngle - 0.25) / obstacleDist
                            elif obstacleAngle > -0.4:
                                obstacleSteering = self.steeringAngle + (obstacleAngle + 0.25) / obstacleDist
                            steer = self.steeringAngle
                            if yellowLineAngle != unKnown:
                                lineFollowingSteering = self.applyPID(yellowLineAngle)
                                if obstacleSteering > 0 and lineFollowingSteering > 0:
                                    steer = max(obstacleSteering, lineFollowingSteering)
                                elif obstacleSteering < 0 and lineFollowingSteering < 0:
                                    steer = min(obstacleSteering, lineFollowingSteering)
                            else:
                                self.needResetPID = True
                            self.setSteeringAngle(steer)
                        elif yellowLineAngle != unKnown:
                            self.setBrakeIntensity(0.0)
                            self.setSteeringAngle(self.applyPID(yellowLineAngle))
                        else:
                            self.setBrakeIntensity(0.4)
                            self.needResetPID = True

            if i % (cfg.param.savePeriod * int(self.controlTime / self.basicTime)) == 0 and cfg.param.sample:
                if cfg.gps.isEnable == 1 and cfg.lidar.isEnable == 1:
                    path = cfg.param.basePath
                    gpsInfo = self.gps.getValues()
                    lidarInfo = self.lidar.getPointCloud(data_type='list')
                    savePointCloudTxt(gpsInfo, lidarInfo, path)
                    self.saveStep += 1
                    if self.saveStep >= cfg.param.sampleStep:
                        sys.exit(997)
            i += 1


class DRLVehicle(Car):
    def __init__(self):
        Car.__init__(self)
        self.saveStep = 0
        self.steeringAngle = 0.0
        self.velocity = cfg.param.velocity
        self.controlTime = cfg.robot.time
        self.basicTime = self.getBasicTimeStep()
        print("#" * 40)
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
            self.firstCall = True
            self.filterSize = 3
            self.cameraChannel = 3
            self.oldCameraValue = [0, 0, 0]
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
            self.HALFAREA = 20
            self.sick = self.getDevice("Sick LMS 291")
            self.sick.enable(cfg.sick.samplingPeriod)
            self.sickWidth = self.sick.getHorizontalResolution()
            self.sickRange = self.sick.getMaxRange()
            self.sickFov = self.sick.getFov()
            print("Sampling Period: ", self.sick.getSamplingPeriod())
            print("Sick Width: ", self.sickWidth)
            print("Sick Range: ", self.sickRange)
            print("Sick Fov: ", self.sickFov)
        print("#" * 40)

        print("PID Enabled: ", cfg.pid.isEnable)
        if cfg.pid.isEnable:
            self.needResetPID = False
            self.oldPIDValue = 0.0
            self.integral = 0.0
            print("KP: ", cfg.pid.KP)
            print("KI: ", cfg.pid.KI)
            print("KD: ", cfg.pid.KD)
        print("#" * 40)

        self.setHazardFlashers(True)
        self.setDippedBeams(True)
        self.setAntifogLights(True)
        self.setWiperMode(False)
        self.setCruisingSpeed(self.velocity)

    def processSickInfo(self, sickInfo):
        sumX = 0
        obstacleDist = 0
        collisionCount = 0
        for i in range(self.HALFAREA * 2):
            key = int(i + self.sickWidth / 2 - self.HALFAREA)
            tmp = sickInfo[key]
            if tmp < 20.0:
                sumX += key
                collisionCount += 1
                obstacleDist += tmp
        if collisionCount == 0:
            return unKnown, obstacleDist
        obstacleDist = obstacleDist / collisionCount
        obstacleAngle = (sumX / collisionCount / self.sickWidth - 0.5) * self.sickFov
        return obstacleAngle, obstacleDist

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

        if yellowPixels == 0:
            return unKnown
        return (xDiffSum / yellowPixels / self.cameraImgWidth - 0.5) * self.cameraImgFov

    def filterAngle(self, newValue):
        if self.firstCall or newValue == unKnown:
            self.firstCall = False
        else:
            for i in range(self.filterSize - 1):
                self.oldCameraValue[i] = self.oldCameraValue[i + 1]
        if newValue == unKnown:
            return unKnown
        else:
            self.oldCameraValue[self.filterSize - 1] = newValue
            filterSum = 0.0
            for i in range(self.filterSize):
                filterSum += self.oldCameraValue[i]
            return filterSum / self.filterSize

    def applyPID(self, yellowLineAngle):
        if self.needResetPID:
            self.oldPIDValue = yellowLineAngle
            self.integral = 0
            self.needResetPID = False
        if self.oldPIDValue * yellowLineAngle < 0:
            self.integral = 0
        if abs(self.integral) < 30:
            self.integral += yellowLineAngle
        diff = yellowLineAngle - self.oldPIDValue
        self.oldPIDValue = yellowLineAngle
        return cfg.pid.KP * yellowLineAngle + cfg.pid.KI * self.integral + cfg.pid.KD * diff

    def run(self):
        i = 1
        while self.step() != -1:
            if i % int(self.controlTime / self.basicTime) == 0:
                if cfg.camera.isEnable:
                    cameraData = self.camera.getImageArray()
                    yellowLineAngle = self.filterAngle(self.processCameraImage(cameraData))
                    if cfg.sick.isEnable:
                        sickData = self.sick.getRangeImage()
                        obstacleAngle, obstacleDist = self.processSickInfo(sickData)
                        if obstacleAngle != unKnown:
                            self.setBrakeIntensity(0.0)
                            obstacleSteering = self.steeringAngle
                            if 0.0 < obstacleAngle < 0.4:
                                obstacleSteering = self.steeringAngle + (obstacleAngle - 0.25) / obstacleDist
                            elif obstacleAngle > -0.4:
                                obstacleSteering = self.steeringAngle + (obstacleAngle + 0.25) / obstacleDist
                            steer = self.steeringAngle
                            if yellowLineAngle != unKnown:
                                lineFollowingSteering = self.applyPID(yellowLineAngle)
                                if obstacleSteering > 0 and lineFollowingSteering > 0:
                                    steer = max(obstacleSteering, lineFollowingSteering)
                                elif obstacleSteering < 0 and lineFollowingSteering < 0:
                                    steer = min(obstacleSteering, lineFollowingSteering)
                            else:
                                self.needResetPID = True
                            self.setSteeringAngle(steer)
                        elif yellowLineAngle != unKnown:
                            self.setBrakeIntensity(0.0)
                            self.setSteeringAngle(self.applyPID(yellowLineAngle))
                        else:
                            self.setBrakeIntensity(0.4)
                            self.needResetPID = True


BmwX5 = DRLVehicle()
BmwX5.run()
