import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as R
import os
import sys
import pandas as pd
sys.path.insert(1, os.path.join(sys.path[0], '../functions'))
from InLocCIIRC_utils.load_CIIRC_transformation.load_CIIRC_transformation import load_CIIRC_transformation

# columns are considered as the bases
def plot_csystem(ax, base, origin, name, color):
    axisNames = ['x', 'y', 'z']
    for i in range(base.shape[1]):
        ax.quiver(origin[0,0], origin[1,0], origin[2,0], base[0,i], base[1,i], base[2,i],
                    color=color, label=name)
        ax.text3D(origin[0,0]+base[0,i], origin[1,0]+base[1,i], origin[2,0]+base[2,i], '%s_%s' % (name, axisNames[i]) )

def visualizePoseSequence(posesDir, queryIds, color, name):
    for queryId in queryIds:
        posePath = os.path.join(posesDir, f'{queryId}.txt')
        P = load_CIIRC_transformation(posePath)
        bases = P[0:3,0:3]
        origin = np.reshape(P[0:3,3], (3,1))
        origin = -np.linalg.inv(bases) @ origin
        bases = bases.T
        # shorten the bases in order to decrease clutter
        #for i in range(3):
        #    bases[:,i] /= 4
        plot_csystem(ax, bases, origin, f'{name}{queryId}', color)

if __name__ == "__main__":
    fig = plt.figure()
    ax = mplot3d.axes3d.Axes3D(fig)
    resolution = 1.4
    ax.set_xlim(-resolution, resolution)
    ax.set_ylim(-resolution, resolution)
    ax.set_zlim(-resolution, resolution)

    modelOrigin = np.reshape(np.array([0.0, 0.0, 0.0]), (3,1))
    modelBases = np.eye(3)
    plot_csystem(ax, modelBases, modelOrigin, 'model', 'black')

    viconOrigin = np.reshape(np.array([-0.13, 0.04, 2.80]), (3,1))
    viconBases = R.from_euler('XYZ', [90.0, 180.0, 0.0], degrees=True).as_matrix().T
    plot_csystem(ax, viconBases, viconOrigin, 'Vicon', 'black')

    queryDir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset/query-HoloLens1'
    referencePosesDir = os.path.join(queryDir, 'poses')
    holoLensPosesDir = os.path.join(queryDir, 'HoloLensPosesDelay4') # w.r.t. model CS
    descriptionsPath = os.path.join(queryDir, 'descriptions.csv')

    descriptionsDf = pd.read_csv(descriptionsPath, sep=' ')
    queryIds = descriptionsDf['id']

    queryIds = np.array([50, 70, 94]) # only a subset, to reduce clutter
    visualizePoseSequence(referencePosesDir, queryIds, 'blue', '')
    visualizePoseSequence(holoLensPosesDir, queryIds, 'red', '')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(-180, -90)
    plt.show()