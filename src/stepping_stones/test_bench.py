'''
Purpose:
Read a recorded .npy file with XYZ coordinates and plot the PSM's path
'''

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import pickle 

# the filename without any directories or extensions
filename = "pos_rec_obstacle"

# plot_all() plots the original and rotated corners, as well as the rotated path
def plot_all(path, img_file, plot_title):
    # initialise the plot
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # make sure the points don't override the plane
    ax = plt.gca()

    plt.axis('equal')

    # add axes limits
    # plt.xlim([-0.20,0.500])
    # plt.ylim([-0.50,0.500])

    # plot the final path in blue
    for point in path:
        ax.scatter(point[0], point[1], point[2], color='blue',linewidth=5.0)
    
    # plot a spherical obstacle
    # ax = fig.add_subplot(111, projection='3d')
    # Make data
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = 0.05 + (0.025 * np.outer(np.cos(u), np.sin(v)))
    y = 0.05 + (0.025 * np.outer(np.sin(u), np.sin(v)))
    z = (0.025 * np.outer(np.ones(np.size(u)), np.cos(v)))
    # Plot the surface
    ax.plot_surface(x, y, z, color='green')

    # include a title if provided
    plt.title("Trajectory Recording with Obstacle Present")
    
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    # plt.savefig('images/pos_rec_obstacle.png')  
    # plt.savefig(img_file)
    plt.show()

def main(filename, plot_title=''):
    print "About to plot", filename
    # convert filename into npy path
    npy_file = "./files/"+filename+".npy"
    # convert filename into png path for saving final result
    img_file = "./images/"+filename+".png"
    REC = np.load(npy_file, allow_pickle=True)

    # convert 2D to 3D with constant Z
    three_d = []
    for coord in REC:
        new_coord = [coord[0], coord[1], 0]
        three_d.append(new_coord)
    
    plot_all(three_d, img_file, plot_title)

if __name__ == '__main__':
    main(filename, plot_title="")

    
