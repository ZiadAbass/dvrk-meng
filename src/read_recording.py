'''
Purpose:
Read a recorded .npy file with XYZ coordinates and plot the PSM's path
'''

import numpy as np
from matplotlib import pyplot as plt
import time
import pickle 

# the filename without any directories or extensions
filename = "pos_rec_no_obstacle"

# plot_all() plots the original and rotated corners, as well as the rotated path
def plot_all(path, img_file, plot_title):
    # initialise the plot
    ax = plt.gca()
    plt.axis('equal')

    # add axes limits
    # plt.xlim([-0.20,0.500])
    # plt.ylim([-0.50,0.500])

    # plot the final path in blue
    for point in path:
        ax.scatter(point[0], point[1], color='blue',linewidth=3.0)
    
    # plot a spherical obstacle
    # circle1 = plt.Circle((0.05, 0.05), 0.025, color='green', fill=True)
    # ax.add_patch(circle1)

    # include a title if provided
    if plot_title != '':
        plt.title(plot_title)

    # plt.savefig('images/pos_rec_obstacle.png')  
    plt.savefig(img_file)

def main(filename, plot_title=''):
    print "About to plot", filename
    # convert filename into npy path
    npy_file = "./files/"+filename+".npy"
    # convert filename into png path for saving final result
    img_file = "./images/"+filename+".png"
    REC = np.load(npy_file, allow_pickle=True)
    plot_all(REC, img_file, plot_title)

if __name__ == '__main__':
    main(filename, plot_title="")

    
