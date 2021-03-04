'''
This script finds the 4 corners of a rectangle aroung a given end effector XY coordinate.
It also takes an angle to rotate that rectangle by.
The rotation happens about the XY coordinate of the end effector's location.
Produces a graphical representation of the result
'''

import math
import matplotlib.pyplot as plt

# specs of bounding rectangle
rec_wid = 150
rec_height = 80
# how far along the length the gripper should be (0-1)
gripper_disp = 0.3
gl = [15,20]    # gripper_location
rotation_in_degrees = 30   # how much to rotate by (anticlockwise, 0 degrees is looking east)

# Rotate a given point by a given angle around a given origin (anticlockwise direction)
def rotate(point, angle, origin):
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def get_bounding_rectangle(gripper_location, degrees):
    # find the corners of the original box before rotating
    ll = (gl[0]-(gripper_disp*rec_wid), gl[1]-(0.5*rec_height))
    ul = (gl[0]-(gripper_disp*rec_wid), gl[1]+(0.5*rec_height))
    ur = (gl[0]+((1-gripper_disp)*rec_wid), gl[1]+(0.5*rec_height))
    lr = (gl[0]+((1-gripper_disp)*rec_wid), gl[1]-(0.5*rec_height))
    original = [ll,ul,ur,lr]

    # convert angle of rotation from degrees to radians
    theta = math.radians(degrees)

    # Apply rotation to each point about the location of the end effector
    rotated = []
    for point in original:
        temp_rotated = rotate(point, theta, gl)
        rotated.append(temp_rotated)
    
    return original, rotated



if __name__ == '__main__':
    # obtain the rectangles' bounds
    original_corners, rotated_corners = get_bounding_rectangle(gripper_location=gl,degrees=rotation_in_degrees)

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

    plt.savefig('hamada.png')    

