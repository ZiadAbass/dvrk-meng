'''
Script for obtaining a search path in a given rectangle.
The rectangle is not rotated at all for now, 
as we cen generate the path and then translate/rotate it according to where the end effector is
'''

import math
import matplotlib.pyplot as plt

# number of search column 
sc = 5
# dimensions of the rectangle
width = 100
height = 40


def get_search_path(ll,lr,ur,ul):
    # find size of the gaps between the columns (column spacing)
    cs = width/(sc+5)
    # get the first search coord in the rectangle (first coordinate)
    fc = [ll[0] + cs, ll[1]+(0.2*height)]
    
    # TODO: continue generating the path

    path = [fc]
    return path

if __name__ == '__main__':
    
    ll = [0,0]
    lr = [width,0]
    ur = [width,height]
    ul = [0,height]

    corners = [ll,lr,ur,ul]

    path = get_search_path(ll,lr,ur,ul)
    # ------- PLOT -------------------

    # initialise the plot
    ax = plt.gca()
    plt.axis('equal')

    # plot the original corners in red
    for point in corners:
        ax.scatter(point[0], point[1], color='red',linewidth=10.0)
    
    # plot the path in green
    for point in path:
        ax.scatter(point[0], point[1], color='green',linewidth=10.0)
    
    plt.savefig('hamada.png') 