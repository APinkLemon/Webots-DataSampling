# -*- coding:utf-8 -*-
"""
作者：34995
日期：2021年03月18日
"""

from dataProcess import *


def RemoveGround(pc, distance_threshold=0.3, sample_size=3, max_iterations=300):  # iteration and time, graph
    i = 0
    random.seed(1234)
    max_point_num = -999
    R_L = range(pc.shape[0])
    max_pc_ground = np.empty([0, 4])
    pc3d = pc[:, 0:3]
    # print(np.mean(pc3d, axis=0))

    while i < max_iterations:
        s3 = random.sample(R_L, sample_size)
        coeffs = estimate_plane(pc3d[s3, :], normalize=False)  # 计算平面方程系数

        if coeffs is None:
            continue

        # 计算平面法向量的模
        r = np.sqrt(coeffs[0] ** 2 + coeffs[1] ** 2 + coeffs[2] ** 2)
        # 若平面法向量与Z轴的夹角大于45度则可能为墙壁，剔除这种情况
        if math.acos(abs(coeffs[2]) / r) > math.pi / 4:
            continue

        # 计算每个点和平面的距离，距离小于阈值的点作为平面上的点
        d = np.divide(np.abs(np.matmul(coeffs[:3], pc3d.T) + coeffs[3]), r)
        near_point_num = pc[np.array(d < distance_threshold), :].shape[0]

        coeffs2 = np.copy(coeffs)
        if coeffs[2] < 0:
            coeffs2[3] = coeffs[3] + distance_threshold * r
            d = np.matmul(coeffs2[:3], pc3d.T) + coeffs2[3]
            pc_ground = pc[np.array(d >= 0), :]
            pc_rmground = pc[np.array(d < 0), :]
        else:
            coeffs2[3] = coeffs[3] - distance_threshold * r
            d = np.matmul(coeffs2[:3], pc3d.T) + coeffs2[3]
            pc_ground = pc[np.array(d < 0), :]
            pc_rmground = pc[np.array(d >= 0), :]
        # 选出内点数最多的平面

        if near_point_num > max_point_num:
            max_coeffs = coeffs
            max_point_num = near_point_num
            max_pc_ground = pc_ground
            max_pc_rmground = pc_rmground

        i = i + 1
    # print(max_pc_rmground)
    return max_pc_rmground, max_pc_ground


def rmGround(data_path, seq):
    start_time = time.time()
    # print('start run seq'+str(seq))
    PointCloudDir = data_path + "velodyne/" + "%04d/" % seq
    OutputDir = data_path + "rmground_points/" + "%04d/" % seq
    if not os.path.exists(OutputDir):
        os.makedirs(OutputDir)

    frames = os.listdir(PointCloudDir)

    # for frame in range(0,bin_num-FrameGap):
    # for frame in np.linspace(0,bin_num-FrameGap,10,endpoint=False).astype(int):
    for frame in frames:
        bin_path = PointCloudDir + frame
        output_bin_path = OutputDir + frame
        pc = np.fromfile(bin_path, dtype=np.float32).reshape([-1, 4])
        ##remove ground
        pc_rmground, pc_ground = RemoveGround(pc)
        # kitti_utils.show_pc(pc_rmground[:, 0:3], np.array([255,255,255]))

        pc_rmground.tofile(output_bin_path)
        print("finish " + str(seq) + "-" + str(frame))
    time_interval = time.time() - start_time
    print('Seq:', str(seq), 'time per frame:', time_interval / len(frames))


def estimate_plane(xyz, normalize=True):
    vector1 = xyz[1, :] - xyz[0, :]
    vector2 = xyz[2, :] - xyz[0, :]

    # 判断vector1是否为0
    if not np.all(vector1):
        return None
    # 共线性检查,如果vector1和vector2三维同比例，则三点共线
    dy1dy2 = vector2 / vector1
    if not ((dy1dy2[0] != dy1dy2[1]) or (dy1dy2[2] != dy1dy2[1])):
        return None

    a = (vector1[1] * vector2[2]) - (vector1[2] * vector2[1])
    b = (vector1[2] * vector2[0]) - (vector1[0] * vector2[2])
    c = (vector1[0] * vector2[1]) - (vector1[1] * vector2[0])

    # normalize
    if normalize:
        r = math.sqrt(a ** 2 + b ** 2 + c ** 2)
        a = a / r
        b = b / r
        c = c / r
    d = -(a * xyz[0, 0] + b * xyz[0, 1] + c * xyz[0, 2])
    return np.array([a, b, c, d])


def normalizePcd(pointCloud):
    pcd = pointCloud
    aabb = pcd.get_axis_aligned_bounding_box()  # o3d.geometry.KDTreeSearchParamHybrid(...)
    center = aabb.get_center()
    extent = aabb.get_extent()
    print(extent)
    maxExtend = max(extent[0], extent[1], extent[2])
    pcd = pcd.translate(-center)
    pcd.scale(1 / maxExtend, center=[0.0, 0.0, 0.0])
    return pcd


def downPcdVoxel(pointCloud, targetNum=4096, render=False):
    pcd = pointCloud
    voxel_size = 1
    shape = np.asarray(pcd.points).shape
    print(shape)
    while shape[0] > targetNum:
        if voxel_size <= 0:
            print("This is a down voxel fail! ")
            return 999
        pcd = o3d.geometry.PointCloud.voxel_down_sample(pcd, voxel_size=voxel_size)
        shape = np.asarray(pcd.points).shape
        if voxel_size > 0:
            voxel_size -= 0.01
        else:
            break
        if render:
            print(np.asarray(pcd.points).shape)
    newNp = np.concatenate((np.asarray(pcd.points), np.zeros((targetNum - np.asarray(pcd.points).shape[0], 3))), 0)
    print(newNp.shape)
    newPcd = normalizePcd(npyToPointCloud(newNp))
    return newPcd


def rotatePointCloud(pointCloud, xyzRotation=(np.pi / 2, 0, 0), initPoint=(0, 0, 0)):
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    R = mesh.get_rotation_matrix_from_xyz(xyzRotation)
    return pointCloud.rotate(R, center=initPoint)


if __name__ == '__main__':
    base = cfg.param.basePath
    base = pathToNpyPath(base)
    fileList = getFilePathList(base)
    print(len(fileList))
    failNum = 0
    failList = []
    for i in range(0, len(fileList)):
        print("#" * 150)
        print(i)
        file = fileList[i]
        print(file)
        a = np.load(file)
        exp = npyToPointCloud(a)
        exp = rotatePointCloud(exp)
        exp = pointCloudToNpy(exp)
        b, c = RemoveGround(exp)
        exp = npyToPointCloud(b)
        newExp = downPcdVoxel(exp)
        if newExp == 999:
            failNum += 1
            failList.append(i)
            print(failNum)
            print(failList)
            # import sys
            # sys.exit(999)
            continue
        exp = pointCloudToNpy(newExp)
        savePath = "dataTrain" + str(cfg.param.trainNum) + fileList[i][11:]
        if cfg.mode == "eval":
            savePath = "dataEvaluate" + str(cfg.param.evalNum) + fileList[i][11:]
        print(savePath)
        np.save(savePath, exp)
    print(failNum)
    print(failList)
