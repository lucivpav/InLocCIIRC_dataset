import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def plot_csystem(ax, base, origin, name, color):
    axisNames = ['x', 'y', 'z']
    for i in range(base.shape[1]):
        ax.quiver(origin[0,0], origin[1,0], origin[2,0], base[0,i], base[1,i], base[2,i],
                    color=color, label=name)
        ax.text3D(origin[0,0]+base[0,i], origin[1,0]+base[1,i], origin[2,0]+base[2,i], '%s_%s' % (name, axisNames[i]) )

if __name__ == "__main__":
    from scipy.spatial.transform import Rotation as R
    rotationMatrix = R.from_euler('x', -110.7, degrees=True).as_matrix()
    #rotationMatrix2 = R.from_euler('xyz', [110.7, 0.0, 0.0], degrees=True).as_matrix() # equivalent to rotationMatrix
    rotationMatrix2 = np.array([[1.00, 0.00, 0.00], [0.00, -0.35, 0.94], [0.00, -0.94, -0.35]])
    xxx = rotationMatrix @ np.linalg.inv(rotationMatrix2)

    cameraCS = np.array([[0.76, -0.47, 0.46], [0.19, 0.82, 0.54], [-0.63, -0.32, 0.71]]) # preHack, pointing y
    yyy = cameraCS @ rotationMatrix 

    fig = plt.figure()
    ax = mplot3d.axes3d.Axes3D(fig)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)

    origin = np.reshape(np.array([0.0, 0.0, 0.0]), (3,1))
    plot_csystem(ax, np.eye(3), origin, 'delta', 'black')

    ### query 40

    markerCS = np.array([[-0.77, 0.22, -0.60], [-0.61, -0.53, 0.60], [-0.19, 0.82, 0.54]])
    #plot_csystem(ax, markerCS, origin, 'markerCSviconDiryPreFix', 'blue')

    markerCS = np.array([[-0.77, 0.46, -0.44], [-0.61, -0.33, 0.72], [0.19, 0.82, 0.54]])
    #plot_csystem(ax, markerCS, origin, 'markerCSviconDiry', 'pink')

    cameraCS = np.array([[-0.76, 0.47, -0.46], [-0.63, -0.32, 0.71], [0.19, 0.82, 0.54]]) # w.r.t. vicon
    #plot_csystem(ax, cameraCS, origin, 'cameraCSMarkerDiry', 'yellow')

    cameraCS = np.array([[-0.16, -0.99, -0.00], [0.03, -0.01, 1.00], [-0.99, 0.16, 0.03]]) # w.r.t. model, camera points to y, master
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryMaster', 'green')

    # w.r.t. model, camera points to y, cameraCS, preRfix
    cameraCS = np.array([[0.76, -0.47, 0.46], [0.19, 0.82, 0.54], [-0.63, -0.32, 0.71]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryPrerFix', 'yellow')

    # w.r.t. model, camera points to y, cameraCS, rFixed, used to projectPointCloud
    cameraCS = np.array([[0.76, -0.46, -0.47], [0.19, -0.54, 0.82], [-0.63, -0.71, -0.32]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryRfixed', 'yellow')

    # w.r.t. model, camera points to -z, cameraCS2, used for RotDist
    cameraCS = np.array([[0.76, 0.46, 0.47], [-0.19, -0.54, 0.82], [-0.63, 0.71, 0.32]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCS2ModelDir-z', 'pink')

    cameraCS = np.array([[0.76, -0.47, 0.46], [0.19, 0.82, 0.54], [-0.63, -0.32, 0.71]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCSfixedDir-z', 'brown')

    cutoutR = np.array([[0.72, -0.35, 0.61], [-0.00, 0.87, 0.50], [-0.70, -0.36, 0.62]])
    #plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')

    #cameraCS = np.array([[0.76, -0.26, -0.60], [0.19, -0.79, 0.58], [-0.63, -0.55, -0.55]]) # w.r.t. model, camera points to y, after hack
    cameraCS = np.array([[0.76, -0.47, 0.46], [-0.19, -0.82, -0.54], [-0.63, -0.32, 0.71]]) # w.r.t. model, camera points to y, after hack2
    print('post hack:')
    #print(cameraCS)
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiry', 'purple')

    cameraCS = np.array([[0.76, 0.46, 0.47], [-0.19, -0.54, 0.82], [-0.63, 0.71, 0.32]]) # w.r.t. model, camera points to -z
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDir-z', 'red')

    ### query 31

    # w.r.t. model, camera points to y, cameraCS, preRfix
    cameraCS = np.array([[-0.14, -0.95, 0.26], [-0.20, 0.29, 0.94], [-0.97, 0.07, -0.23]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryPrerFix', 'yellow')

    cutoutR = np.array([[-0.26, 0.48, 0.83], [0.00, 0.86, -0.50], [-0.97, -0.13, -0.23]])
    #plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')

    ### query 3

    # w.r.t. model, camera points to -z, cameraCS2
    cameraCS = np.array([[0.15, 0.99, -0.00], [-0.02, 0.01, 1.00], [0.99, -0.15, 0.02]]) 
    plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryPrerFix', 'yellow')

    cutoutR = np.array([[0.26, 0.00, -0.97], [-0.00, 1.00, 0.00], [0.97, -0.00, 0.26]])
    plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')



    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(-180, -90)
    plt.show()