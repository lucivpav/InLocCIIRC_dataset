import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import scipy.io as sio
import open3d as o3d
import sys
import os
from Equirec2Perspec.Equirec2Perspec import Equirectangular
import cv2

def buildXYZcut(mesh, f, camera_position, camera_rotation, sensorSize):
    # In OpenGL, camera points toward -z by default, hence we don't need rFix like in the MATLAB code
    sensorWidth = sensorSize[0]
    sensorHeight = sensorSize[1]
    fovHorizontal = 2*np.arctan((sensorWidth/2)/f)
    fovVertical = 2*np.arctan((sensorHeight/2)/f)

    scene = pyrender.Scene()
    scene.add(mesh)
    camera = pyrender.PerspectiveCamera(fovVertical)

    rotation_matrix = R.from_euler('xyz', camera_rotation, degrees=True).as_matrix()
    camera_pose = np.eye(4)
    camera_pose[0:3,0:3] = rotation_matrix
    camera_pose[0:3,3] = camera_position
    scene.add(camera, pose=camera_pose)

    light = pyrender.SpotLight(color=np.ones(3), intensity=30.0,
                                innerConeAngle=np.pi/16.0,
                                outerConeAngle=np.pi/2.0)
    scene.add(light, pose=camera_pose)

    r = pyrender.OffscreenRenderer(sensorWidth, sensorHeight)
    #depth = r.render(scene, pyrender.constants.RenderFlags.DEPTH_ONLY)
    depth, actualDepth = r.render(scene)
    #plt.figure()
    #plt.axis('off')
    #plt.imshow(depth, cmap=plt.cm.gray_r)
    #plt.show()

    # XYZ cut
    scaling = 1.0/f

    cameraDirection = np.eye(3)
    cameraDirection[2,2] = -1 # make camera point toward -z by default, as in OpenGL

    sensorCoordinateSystem = np.matmul(rotation_matrix, cameraDirection)
    sensorXAxis = sensorCoordinateSystem[0,:]
    sensorYAxis = -sensorCoordinateSystem[1,:]
    cameraDirection = sensorCoordinateSystem[2,:] # unit vector

    xyzCut = np.zeros((sensorHeight, sensorWidth, 3))
    #pts = []

    for x in range(-int(sensorWidth/2), int(sensorWidth/2)):
        for y in range(-int(sensorHeight/2), int(sensorHeight/2)):
            sensorPoint = camera_position + cameraDirection + \
                            x * scaling * sensorXAxis + \
                            y * scaling * sensorYAxis
            imageX = x + int(sensorWidth/2)
            imageY = y + int(sensorHeight/2)
            d = depth[imageY, imageX]
            sensorPointDir = sensorPoint - camera_position
            intersectedPoint = camera_position + sensorPointDir * d
            xyzCut[imageY, imageX, :] = intersectedPoint
            #pts.append(intersectedPoint)
    
    #pts = np.array(pts)
    #pcd = o3d.geometry.PointCloud()
    #pcd.points = o3d.utility.Vector3dVector(pts)
    #o3d.visualization.draw_geometries([pcd])

    return xyzCut, depth

def getSweepRecord(sweepData, panoId):
    for i in range(len(sweepData)):
        sweepRecord = sweepData[i]
        if sweepRecord['panoId'] == panoId:
            return sweepRecord

if __name__ == '__main__':
    cutoutsDir = '/Volumes/Elements/CIIRC/cutouts'
    panoramasDir = '/Volumes/Elements/CIIRC/rotatedPanoramas'
    meshPath = '/Volumes/Elements/CIIRC/matterpak_sehs6V3VnSW/mesh - rotated.ply'
    sweepDataPath = '/Volumes/Elements/CIIRC/sweepData.mat'
    panoIds = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, \
                16, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, \
                32, 33, 35, 36, 37]
    panoIds = [2, 3]
    f = 600
    sensorSize = np.array([1600, 1200])
    sensorWidth = sensorSize[0]
    sensorHeight = sensorSize[1]
    fovHorizontal = 2*np.arctan((sensorWidth/2)/f)
    fovHorizontal = np.rad2deg(fovHorizontal)
    fovVertical = 2*np.arctan((sensorHeight/2)/f)
    fovVertical = np.rad2deg(fovVertical)

    sweepData = sio.loadmat(sweepDataPath, squeeze_me=True)['sweepData']
    trimesh_model = trimesh.load(meshPath)
    mesh = pyrender.Mesh.from_trimesh(trimesh_model)

    for panoId in panoIds:
        sweepRecord = getSweepRecord(sweepData, panoId)

        thisPanoCutoutsDir = os.path.join(cutoutsDir, str(panoId))
        if not os.path.isdir(thisPanoCutoutsDir):
            os.mkdir(thisPanoCutoutsDir)

        panoramaPath = os.path.join(panoramasDir, str(panoId) + '.jpg')
        equ = Equirectangular(panoramaPath)

        xh = np.arange(-180, 180, 30)
        yh0 = np.zeros((len(xh)))
        yhTop = yh0 + 30
        yhBottom = yh0 - 30

        x = np.concatenate((xh, xh, xh))
        y = np.concatenate((yh0, yhTop, yhBottom))

        for i in range(len(x)):
            #yaw = x[i]
            #pitch = y[i]
            yaw = 0
            pitch = 0
            panoramaProjection = equ.GetPerspective(fovHorizontal, yaw, pitch, sensorHeight, sensorWidth)
            filename = 'cutout_%d_%d_%d.jpg' % (panoId, yaw, pitch)
            path = os.path.join(thisPanoCutoutsDir, filename)
            cv2.imwrite(path, panoramaProjection) # this can freeze
            # set up the mat file
            cameraRotation = sweepRecord['rotation'] + np.array([pitch, yaw, 0.0])
            XYZcut, depth = buildXYZcut(mesh, f, sweepRecord['position'], cameraRotation, sensorSize)
            filename = filename + '.mat'
            path = os.path.join(thisPanoCutoutsDir, filename)
            sio.savemat(path, {'RGBcut': panoramaProjection, 'XYZcut': XYZcut, 'depth': depth})
            filename = 'depth_%d_%d_%d.jpg' % (panoId, yaw, pitch)
            path = os.path.join(thisPanoCutoutsDir, filename)
            cv2.imwrite(path, depth)
            #plt.imsave(path, depth, cmap=plt.cm.gray_r)
            exit(0)