'''
This script constructs a rotated rectangle around a given end effector location and orientation. 
A rotated search path is also produced which the PSM would follow as it searches for the vessel inside the box. 
Resulting rectangle and path are plotted.
Inputs: 
    - XY location of the end effector
    - orientation of the end effector
    - dimension of the required bounding rectangle
    - [optional] how far along the rectangle we want the gripper to be (gripper_disp)
    - [optional] padding area on the inside of the height (0-1) (height_padding)
    - [optional] number of search columns along the rectangle's width (search_columns)
Process: 
    - Generates a bounding box around it
    - Generates a path inside the box
    - Rotates the box and path about the XY location of the end effector
Outputs:
    - Coords of bounding rectangle
    - Coords of the path points inside rectangle
    - colour-coded plots of these outputs
'''

import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Rotate a given point by a given angle around a given origin (anticlockwise direction)
def rotate(point, angle, origin):
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

# rotate_all() takes in the corners and path for a rectangle that has not been rotated yet
# it rotates all the points about the position of the end effector anticlockwise by the given angle in degrees
def rotate_all(corners, path, degrees):
    # convert angle of rotation from degrees to radians
    theta = math.radians(degrees)

    # Apply rotation to each point about the location of the end effector
    corners_rotated = []
    for point in corners:
        temp_rotated = rotate(point, theta, gl)
        corners_rotated.append(temp_rotated)
    path_rotated = []
    for point in path:
        temp_rotated = rotate(point, theta, gl)
        path_rotated.append(temp_rotated)
    
    return corners_rotated, path_rotated

# get_bounding_rectangle() finds the corners for a rectangle around a given end effect 2D coordinate
# can optionally provide gripper_disp, defines how far along the length the gripper should be (0-1)
def get_bounding_rectangle(gripper_location,gripper_disp):
    # find the corners of the original box before rotating
    ll = (gl[0]-(gripper_disp*rec_wid), gl[1]-(0.5*rec_height))
    ul = (gl[0]-(gripper_disp*rec_wid), gl[1]+(0.5*rec_height))
    ur = (gl[0]+((1-gripper_disp)*rec_wid), gl[1]+(0.5*rec_height))
    lr = (gl[0]+((1-gripper_disp)*rec_wid), gl[1]-(0.5*rec_height))
    original = [ll,lr,ur,ul]    
    return original

# get_search_path() takes in 4 corner coords for a rectangle that is not rotated
# search_columns: number of search columns along the rectangle's width
# height_padding: padding area on the inside of the height (0-1)
def get_search_path(corners, search_columns, height_padding):
    # extract individual corner locations
    ll,lr,ur,ul = corners
    
    # find size of the gaps between the columns (column spacing)
    cs = rec_wid/(search_columns+1)

    path = []
    for col in range(1,search_columns+1):    
        # get the first search coord in the rectangle (first coordinate)
        first = [ll[0] + (cs*col), ll[1]+(0.2*rec_height)]
        second = [ll[0] + (cs*col), ll[1]+((1-height_padding)*rec_height)]
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

# plot_all() plots the original and rotated corners, as well as the rotated path
def plot_all(original_corners, rotated_corners, rotated_path):
    # initialise the plot
    ax = plt.gca()
    plt.axis('equal')
    plt.xlim([-0.20,0.500])
    plt.ylim([-0.50,0.500])

    # plot the gripper location in red
    ax.scatter(gl[0], gl[1], color='red',linewidth=3.0)
    
    # plot the final path in brown
    for point in rotated_path:
        ax.scatter(point[0], point[1], color='brown',linewidth=3.0)
    
    # draw the original rectangle
    for coord_idx in range (0,len(original_corners)):
        if coord_idx == len(original_corners)-1:
            x_values = [original_corners[coord_idx][0], original_corners[0][0]]
            y_values = [original_corners[coord_idx][1], original_corners[0][1]]
        else:
            x_values = [original_corners[coord_idx][0], original_corners[coord_idx+1][0]]
            y_values = [original_corners[coord_idx][1], original_corners[coord_idx+1][1]]
        plt.plot(x_values, y_values, '--',linewidth=1.5,zorder=1,color='green')

    # draw the rotated rectangle
    for coord_idx in range (0,len(rotated_corners)):
        if coord_idx == len(rotated_corners)-1:
            x_values = [rotated_corners[coord_idx][0], rotated_corners[0][0]]
            y_values = [rotated_corners[coord_idx][1], rotated_corners[0][1]]
        else:
            x_values = [rotated_corners[coord_idx][0], rotated_corners[coord_idx+1][0]]
            y_values = [rotated_corners[coord_idx][1], rotated_corners[coord_idx+1][1]]
        plt.plot(x_values, y_values,linewidth=3.5,zorder=1,color='blue')

    plt.savefig('hamada.png')   


def main(rec_width_in,rec_height_in, gripper_location, gripper_rotation_degrees, gripper_displacement=0.3, search_columns=4, height_padding=0.2, plot=True):
    global rec_wid
    global rec_height
    global gl
    global rotation_in_degrees

    # dimensions of bounding rectangle
    rec_wid = rec_width_in
    rec_height = rec_height_in
    # gripper_location
    gl = gripper_location
    # how much to rotate by (anticlockwise, 0 degrees is looking east)
    rotation_in_degrees = gripper_rotation_degrees

    print "gl", gl
    print "rec_wid", rec_wid

    # obtain the rectangles' bounds
    original_corners = get_bounding_rectangle(gripper_location=gl,gripper_disp=gripper_displacement)

    # find the search path inside the rectangle
    path = get_search_path(original_corners,search_columns=search_columns, height_padding=height_padding)

    # rotate according to the end effector's orientation
    rotated_corners, rotated_path = rotate_all(original_corners, path, degrees=rotation_in_degrees)

    if plot:
        # plot the original and rotated corners, as well as the rotated path
        plot_all(original_corners, rotated_corners, rotated_path)

    # return the rotated rectangle and the new path
    return rotated_corners, rotated_path



if __name__ == '__main__':

    main(rec_width_in=0.150,rec_height_in=0.080, gripper_location=[0.200,0.200], gripper_rotation_degrees=120)
 

