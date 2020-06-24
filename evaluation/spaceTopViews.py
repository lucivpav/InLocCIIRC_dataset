import numpy as np
import sys
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import scipy.io as sio
import pandas as pd
from adjustText import adjust_text
import matplotlib.patheffects as PathEffects
sys.path.insert(1, os.path.join(sys.path[0], '../functions'))
from InLocCIIRC_utils.projectMesh.projectMesh import projectMesh
from InLocCIIRC_utils.load_CIIRC_transformation.load_CIIRC_transformation  import load_CIIRC_transformation

def project(point, mag, sensorSize, rotationMatrix, translation):
    xmag = mag
    ymag = mag
    xmag = sensorSize[0] / sensorSize[1] * ymag
    P = np.zeros((4,4)) # ortho projection matrix, taken from pyrender
    near = 0.05
    far = 100.0
    P[0][0] = 1.0 / xmag
    P[1][1] = 1.0 / ymag
    P[2][2] = 2.0 / (near - far)
    P[2][3] = (far + near) / (near - far)
    P[3][3] = 1.0

    camera_pose = np.eye(4)
    camera_pose[0:3,0:3] = rotationMatrix
    camera_pose[0:3,3] = translation

    t = np.reshape(translation, (3,1))
    transformedPoint = np.vstack((point, np.array([1.0]).T))
    projectedPoint = P @ (np.linalg.inv(camera_pose) @ transformedPoint)
    projectedPoint = np.array([projectedPoint[0], projectedPoint[1]])
    sensor = np.reshape((sensorSize/2).T, (2,1))
    projectedPoint = np.multiply(projectedPoint, np.multiply(sensor, np.reshape(np.array([1, -1]).T, (2,1)))) + sensor
    return projectedPoint

def plotLocation(text, color, x, y, texts):
    plt.plot(x, y, 'o', color=color, markersize=2)
    textObject = plt.text(x, y+5, text, size=4, ha='center', va='center', color='black')
    textObject.set_path_effects([PathEffects.withStroke(linewidth=1, foreground='white')])
    texts.append(textObject)

f = 1385.6406460551023
datasetDir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset_mirror'
evaluationDir = os.path.join(datasetDir, 'evaluation')
temporaryDir = os.path.join(evaluationDir, 'temporary')
if not os.path.isdir(temporaryDir):
    os.mkdir(temporaryDir)
queryDescriptionsPath = os.path.join(datasetDir, 'query', 'descriptions.csv')
retrievedQueriesPath = os.path.join(evaluationDir, 'retrievedQueries.csv')
spaceNames = ['B-315', 'B-670']
sensorSize = 2*np.array([1600, 1200])
translations = [np.array([3.0, 1.0, 3.0]).T, np.array([-8.0, 1.0, -3.0]).T]
magnifications = [18.0, 12.0]
rotationMatrix = R.from_euler('xyz', np.array([-90.0, 90.0, 0.0]), degrees=True).as_matrix()

# project spaces from top
basicTopViews = []
for i in range(len(spaceNames)):
    spaceName = spaceNames[i]
    topViewPath = os.path.join(temporaryDir, 'topView-%s.png' % spaceName)
    if os.path.exists(topViewPath):
        RGBcut = plt.imread(topViewPath)
        RGBcut = RGBcut.copy()
        basicTopViews.append(RGBcut)
        continue
    meshPath = os.path.join(datasetDir, 'models', spaceName, 'mesh_rotated_noceiling.obj')
    RGBcut, XYZcut, depth = projectMesh(meshPath, f, rotationMatrix, translations[i], sensorSize, True, magnifications[i])
    plt.imsave(topViewPath, RGBcut)
    basicTopViews.append(RGBcut)

texts = []
figsize = sensorSize/100.0
figsize = (12, 8)
figsize = (6.4, 4.8)
dpi = 400

# sweeps
for i in range(len(spaceNames)):
    texts.append([])
    fig = plt.figure(i)
    fig.set_size_inches(figsize)
    fig.set_dpi(dpi)
    plt.imshow(basicTopViews[i])
    sweepDataPath = os.path.join(datasetDir, 'sweepData', '%s.mat' % spaceNames[i])
    sweepData = sio.loadmat(sweepDataPath, squeeze_me=True)['sweepData']
    for j in range(len(sweepData)):
        sweepRecord = sweepData[j]
        sweepId = sweepRecord['panoId']
        sweepT = np.reshape(sweepRecord['position'], (3,1))
        projectedPoint = project(sweepT, magnifications[i], sensorSize, rotationMatrix, translations[i])
        plotLocation(str(sweepId), 'red', projectedPoint[0], projectedPoint[1], texts[i])

# query
df = pd.read_csv(queryDescriptionsPath, sep=' ')
for i in range(len(df)):
    queryId = df['id'][i]
    posePath = os.path.join(datasetDir, 'query', 'poses', '%d.txt' % queryId)
    P = load_CIIRC_transformation(posePath)
    t = -P[0:3,0:3] @ P[0:3,3]
    t = np.reshape(t, (3,1))
    querySpace = df['space'][i]
    queryInMap = df['inMap'][i]
    spaceId = spaceNames.index(querySpace)
    projectedPoint = project(t, magnifications[spaceId], sensorSize, rotationMatrix, translations[spaceId])
    plt.figure(spaceId)
    plotLocation(str(queryId), 'blue', projectedPoint[0], projectedPoint[1], texts[spaceId])

# retrieved query poses
df = pd.read_csv(retrievedQueriesPath, sep=' ')
for i in range(len(df)):
    queryId = df['id'][i]
    posePath = os.path.join(evaluationDir, 'retrievedPoses', '%d.txt' % queryId)
    P = load_CIIRC_transformation(posePath)
    t = -P[0:3,0:3] @ P[0:3,3]
    t = np.reshape(t, (3,1))
    querySpace = df['space'][i]
    if np.any(np.isnan(t)):
        continue
    spaceId = spaceNames.index(querySpace)
    projectedPoint = project(t, magnifications[spaceId], sensorSize, rotationMatrix, translations[spaceId])
    plt.figure(spaceId)
    plotLocation(str(queryId), 'yellow', projectedPoint[0], projectedPoint[1], texts[spaceId])

# try to make texts not overlap
for i in range(len(spaceNames)):
    plt.figure(i)
    adjust_text(texts[i], arrowprops=dict(arrowstyle='-', color='black'), force_points=(5, 5*2.5))

# final save of images
for i in range(len(spaceNames)):
    spaceName = spaceNames[i]
    spaceTopViewPath = os.path.join(evaluationDir, 'topView-%s.png' % spaceName)
    fig = plt.figure(i)
    plt.axis('off')
    fig.savefig(spaceTopViewPath, bbox_inches='tight', pad_inches=0)
plt.axis('on')