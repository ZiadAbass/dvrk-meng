'''
Purpose:
Read a recorded .npy file with XYZ coordinates and plot the PSM's path
'''

import numpy as np
from matplotlib import pyplot as plt
import time
import pickle 

filename = './files/pos_recording_goto.npy'

# plot_all() plots the original and rotated corners, as well as the rotated path
def plot_all(path):
    # initialise the plot
    ax = plt.gca()
    plt.axis('equal')

    # plt.xlim([-0.20,0.500])
    # plt.ylim([-0.50,0.500])

    # plot the final path in blue
    for point in path:
        ax.scatter(point[0], point[1], color='blue',linewidth=3.0)

    plt.savefig('images/traj_20kp.png')  


if __name__ == '__main__':
    REC = np.load(filename, allow_pickle=True)
    
    plot_all(REC)
