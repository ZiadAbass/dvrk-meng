'''
Script for obtaining a search path in a given rectangle. It takes in 4 corners of the rectangle
The rectangle is not rotated at all for now, 
as we cen generate the path and then translate/rotate it according to where the end effector is
'''

import math
import matplotlib.pyplot as plt

# number of search columns
sc = 5
# dimensions of the rectangle
width = 100
height = 40
# padding area on the inside of the height (0-1)
height_padding = 0.2


def get_search_path(ll,lr,ur,ul):
    
    # find size of the gaps between the columns (column spacing)
    cs = width/(sc+1)

    path = []
    for col in range(1,sc+1):    
        # get the first search coord in the rectangle (first coordinate)
        first = [ll[0] + (cs*col), ll[1]+(0.2*height)]
        second = [ll[0] + (cs*col), ll[1]+((1-height_padding)*height)]
        path.append(first)
        path.append(second)

    # change the order of the items in the path such that a trajectory can be generated between them
    # (make the path snake shaped rather than vertical lines)
    ordered_path = []
    switch = False
    ignore = False
    for idx,point in enumerate(path):
        if ignore:
            ignore = False
            continue
        if (idx%2 == 0) and not switch:
            ordered_path.append(point)
            switch = True
            ignore = False
        elif (idx%2 == 0) and switch:
            # in this case, append the next point before this one
            ordered_path.append(path[idx+1])
            ordered_path.append(path[idx])
            ignore = True
            switch = False
        else:
            ordered_path.append(point)
        
    return ordered_path



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