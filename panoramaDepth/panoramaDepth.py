import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import scipy.io as sio
import open3d as o3d

mesh_path = '/Volumes/Elements/CIIRC/matterpak_sehs6V3VnSW/mesh - rotated.ply'
outputPath = '/Volumes/Elements/CIIRC/XYZcuts/bla.mat'
f = 500

# In OpenGL, camera points toward -z by default, hence we don't need rFix like in the MATLAB code
camera_rotation = np.array([0.176829305501008, 3.651460591341834, -0.119347990257635])
camera_position = np.array([9.786105947569013e-04, 1.693258881568909, 0.068662971258163])
sensorWidth = 1000
sensorHeight = 1000
fovHorizontal = 2*np.arctan((sensorWidth/2)/f)
fovVertical = 2*np.arctan((sensorHeight/2)/f)

trimesh_model = trimesh.load(mesh_path)
mesh = pyrender.Mesh.from_trimesh(trimesh_model)
scene = pyrender.Scene()
scene.add(mesh)
camera = pyrender.PerspectiveCamera(fovVertical, aspectRatio=1.0) # TODO: aspect ratio

rotation_matrix = R.from_euler('xyz', camera_rotation, degrees=True).as_matrix()
camera_pose = np.eye(4)
camera_pose[0:3,0:3] = rotation_matrix
camera_pose[0:3,3] = camera_position
scene.add(camera, pose=camera_pose)

r = pyrender.OffscreenRenderer(sensorWidth, sensorHeight)
depth = r.render(scene, pyrender.constants.RenderFlags.DEPTH_ONLY)
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
pts = []

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
        pts.append(intersectedPoint)

sio.savemat(outputPath, {'XYZcut': xyzCut})
pts = np.array(pts)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)
o3d.visualization.draw_geometries([pcd])