import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import scipy.io as sio
import os
import open3d as o3d

datasetDir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset'
spaceName = 'B-315'
pcPath = os.path.join(datasetDir, 'models', spaceName, 'cloud - rotated.ply')
f = 3172.435
sensorWidth = 4032
sensorHeight = 3024
fovVertical = 2*np.arctan((sensorHeight/2)/f)

# make the sensor size smaller in order to cast it onto screen, while keeping the FoV
sensorWidth = sensorWidth / 2
sensorHeight = sensorHeight / 2
f = sensorHeight / (2 * np.tan(fovVertical/2))

pcd = o3d.io.read_point_cloud(pcPath)

rotation_matrix = np.array([[-0.233688694914022, -0.972233919031819, -0.0122800876793087],
                    [0.120006830330971, -0.0413737758952669, 0.991910566201451],
                    [-0.964877170702725, 0.230324591288465, 0.126343294655896]])
#rotation_matrix2 = R.from_euler('xyz', [180.0, 95.0, 0.0], degrees=True).as_matrix()
rotation_matrix2 = R.from_euler('xyz', [-90.0, 00.0, 0.0], degrees=True).as_matrix()
#rotation_matrix = R.from_euler('xyz', [0.0, 90.0, 0.0], degrees=True).as_matrix()
camera_position = np.array([2.28837252516427, 1.52868663468306, 0.629325829247731])

camera_pose = np.eye(4)
camera_pose[0:3,0:3] = np.matmul(rotation_matrix, rotation_matrix2)
#camera_pose[0:3,3] = -camera_position

camera_position_mat = np.eye(4)
camera_position_mat[0:3,3] = -camera_position

pcd.transform(camera_position_mat)
vis = o3d.visualization.Visualizer()
vis.create_window(width=int(sensorWidth), height=int(sensorHeight))
ctr = vis.get_view_control()
vis.add_geometry(pcd)
ro = vis.get_render_option()
ro.point_size = 6.0

intrinsic = o3d.camera.PinholeCameraIntrinsic()
pcp = ctr.convert_to_pinhole_camera_parameters()
pcp.intrinsic.set_intrinsics(int(sensorWidth), int(sensorHeight), f, f, sensorWidth/2-0.5, sensorHeight/2-0.5)
pcp.extrinsic = camera_pose
ctr.convert_from_pinhole_camera_parameters(pcp)
#o3d.visualization.draw_geometries([pcd])
vis.run()
vis.destroy_window()


# son of a bitch, why would the window size have to be equal to the sensor fucking size?