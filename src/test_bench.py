'''
This script combines the rotate_rectangle and find_path scripts
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

# dimensions of bounding rectangle
rec_wid = 150
rec_height = 80

# gripper_location
gl = [15,20]
# how much to rotate by (anticlockwise, 0 degrees is looking east)
rotation_in_degrees = 120

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
def get_bounding_rectangle(gripper_location,gripper_disp=0.3):
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
def get_search_path(corners, search_columns=5, height_padding = 0.2):
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

    # plot the gripper location in red
    ax.scatter(gl[0], gl[1], color='red',linewidth=10.0)

    # plot original rectangle in green
    for point in original_corners:
        ax.scatter(point[0], point[1], color='green',linewidth=10.0)

    # plot rotated rectangle in blue
    for point in rotated_corners:
        ax.scatter(point[0], point[1], color='blue',linewidth=10.0)
    
    # plot the final path in brown
    for point in rotated_path:
        ax.scatter(point[0], point[1], color='brown',linewidth=10.0)

    plt.savefig('hamada.png')   


if __name__ == '__main__':
    # obtain the rectangles' bounds
    original_corners = get_bounding_rectangle(gripper_location=gl)

    # find the search path inside the rectangle
    path = get_search_path(original_corners)

    # rotate according to the end effector's orientation
    rotated_corners, rotated_path = rotate_all(original_corners, path, degrees=rotation_in_degrees)

    # plot the original and rotated corners, as well as the rotated path
    plot_all(original_corners, rotated_corners, rotated_path)
 

