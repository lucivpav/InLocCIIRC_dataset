import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# columns are considered as the bases
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

    # w.r.t. model, y *= -1 applied
    markerCS = np.array([[0.77, -0.46, 0.44], [0.19, 0.82, 0.54], [-0.61, -0.33, 0.72]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'yellow')

    # w.r.t. model, y *= -1 NOT applied
    markerCS = np.array([[0.77, -0.22, 0.60], [-0.19, 0.82, 0.54], [-0.61, -0.53, 0.60]]) 
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'brown')

    cutoutR = np.array([[0.72, -0.35, 0.61], [-0.00, 0.87, 0.50], [-0.70, -0.36, 0.62]])
    #plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')

    #cameraCS = np.array([[0.76, -0.26, -0.60], [0.19, -0.79, 0.58], [-0.63, -0.55, -0.55]]) # w.r.t. model, camera points to y, after hack
    cameraCS = np.array([[0.76, -0.47, 0.46], [-0.19, -0.82, -0.54], [-0.63, -0.32, 0.71]]) # w.r.t. model, camera points to y, after hack2
    #print('post hack:')
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

    # w.r.t. model, y *= -1 applied
    markerCS = np.array([[-0.11, -0.96, 0.27], [-0.20, 0.29, 0.94], [-0.97, 0.05, -0.23]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'yellow')

    # w.r.t. model, y *= -1 NOT applied
    markerCS = np.array([[-0.11, -0.94, 0.32], [0.20, 0.29, 0.94], [-0.97, 0.17, 0.16]]) # above not applied
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'brown')

    ### query 3

    # w.r.t. model, camera points to -z, cameraCS2
    cameraCS = np.array([[0.15, 0.99, -0.00], [-0.02, 0.01, 1.00], [0.99, -0.15, 0.02]]) 
    #plot_csystem(ax, cameraCS, origin, 'cameraCSModelDiryPrerFix', 'yellow')

    cutoutR = np.array([[0.26, 0.00, -0.97], [-0.00, 1.00, 0.00], [0.97, -0.00, 0.26]])
    #plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')


    ### query 36

    # w.r.t. model, y *= -1 applied, ZYX order
    markerCS = np.array([[0.77, -0.24, 0.59], [-0.61, -0.53, 0.58], [0.18, -0.81, -0.56]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'yellow')

    # w.r.t. model, y *= -1 applied, ZYX order, rFixed along xAxis
    markerCS = np.array([[0.77, 0.59, 0.24], [-0.61, 0.58, 0.53], [0.18, -0.56, 0.81]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'yellow')

    # w.r.t. model, y *= -1 NOT applied, ZYX order
    markerCS = np.array([[0.77, 0.57, -0.29], [0.61, -0.53, 0.58], [0.18, -0.62, -0.76]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'brown')

    # w.r.t. model, y *= -1 NOT applied, XYZ order
    # NOTE: although it looks more correct than when y *= -1 is applied, it is in fact still incorrect
    markerCS = np.array([[0.77, 0.18, 0.61], [-0.59, 0.56, 0.58], [-0.24, -0.81, 0.53]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'blue')

    # w.r.t. model, y *= -1 applied, XYZ order
    # NOTE: this is also not correct, although it looks like it could be
    markerCS = np.array([[0.77, 0.18, -0.61], [0.29, 0.76, 0.58], [0.57, -0.62, 0.53]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'brown')

    # w.r.t. model, y *= -1 applied, ZYX order, Y-coordinates flipped
    markerCS = np.array([[0.77, -0.24, 0.59], [0.61, 0.53, -0.58], [0.18, -0.81, -0.56]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'blue')

    # w.r.t. model, camera points to ??? should point to y!
    cameraCS = np.array([[0.77, -0.26, 0.58], [-0.61, -0.53, 0.58], [0.16, -0.80, -0.57]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCS', 'yellow')

    # w.r.t. model, this is fed directly to projectPC,
    cameraCS = np.array([[0.77, -0.58, -0.26], [-0.61, -0.58, -0.53], [0.16, 0.57, -0.80]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCS', 'blue')

    # used for finding the closest cutout
    cameraCS = np.array([[0.77, -0.26, 0.58], [-0.61, -0.53, 0.58], [0.16, -0.80, -0.57]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCS', 'blue')
    # TODO: compare with reference closest cutout in backup

    # w.r.t. model, this is fed directly to projectPC, from master branch
    cameraCS = np.array([[0.77, -0.58, -0.26], [-0.61, -0.58, -0.53], [0.16, 0.57, -0.80]])
    #plot_csystem(ax, cameraCS, origin, 'cameraCSmaster', 'pink')

    cutoutR = np.array([[0.93, -0.18, -0.31], [-0.00, 0.86, -0.50], [0.36, 0.47, 0.81]])
    #plot_csystem(ax, cutoutR, origin, 'cutoutR', 'red')

    # query 1
    markerCS = np.array([[-0.13, -0.99, -0.00], [0.03, -0.01, 1.00], [-0.99, 0.13, 0.03]])
    #plot_csystem(ax, markerCS, origin, 'markerCS', 'blue')

    ### query 32
    temporary = np.array([[0.75, -0.08, -0.65], [-0.09, -1.00, 0.01], [-0.65, 0.05, -0.76]]).T
    #plot_csystem(ax, temporary, origin, 'reference', 'blue')

    temporary = np.array([[0.19, -0.05, -0.98], [-0.17, -0.98, 0.01], [-0.97, 0.17, -0.20]]).T
    #plot_csystem(ax, temporary, origin, 'temporary', 'yellow')
    #plot_csystem(ax, temporary2, origin, 'temporary2', 'pink')



    markerWrtModel = np.array([[0.56, -0.53, -0.64], [0.44, -0.46, 0.77], [-0.70, -0.71, -0.02]])
    origin = np.reshape(np.array([-3.6426, 1.6608, -0.0278]), (3,1))
    plot_csystem(ax, markerWrtModel, origin, 'markerWrtModel', 'yellow')

    cameraWrtModel = np.array([[0.54, -0.39, -0.74], [0.55, -0.50, 0.67], [-0.63, -0.77, -0.06]])
    origin = np.reshape(np.array([-3.7976, 1.3192, -0.4465]), (3,1))
    plot_csystem(ax, cameraWrtModel, origin, 'cameraWrtModel', 'blue')



    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(-180, -90)
    plt.show()