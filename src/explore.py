'''
Purpose: 
- Send arm to a specific 3D coordinate
- Have the arm explore 4 coordinates around it (+/- dX and +/- dY)
'''

import master as mm


# find_exploration_points() uses the PSM's current position and finds the 4 surrounding coordinates
# an 'increment' mm away from the current position (in +/- X and +/- Y)
def find_exploration_points(psm,group,increment):
    # find the arm's position in 3D
    cur_XYZ = mm.read_display_dvrk_pos(psm, verbose=False)
    # convert the increment given from mm to meters
    inc_m = increment/1000

    exploration_coords = []
    # +/- in Y
    exploration_coords.append([cur_XYZ[0],cur_XYZ[1]+inc_m,cur_XYZ[2]])
    exploration_coords.append([cur_XYZ[0],cur_XYZ[1]-inc_m,cur_XYZ[2]])
    # -/+ in X
    exploration_coords.append([cur_XYZ[0]-inc_m,cur_XYZ[1],cur_XYZ[2]])
    exploration_coords.append([cur_XYZ[0]+inc_m,cur_XYZ[1],cur_XYZ[2]])

    return exploration_coords


if __name__ == '__main__':
    # initialise and home
    psm,group = mm.init_and_home()

    # find the coordinates of the 4 exploration points
    exp_coords = find_exploration_points(psm,group,increment=15)

    # go to all 4
    for idx, target_coord in enumerate(exp_coords):
        buf = "Press Enter to move to coordinate number %d" % (idx)
        raw_input(buf)
        mm.goto_xyz(psm,target_coord,6000,group)

    psm.shutdown()



'''
The /dvrk/PSM1/position_cartesian_current topic message type is PoseStamped
problem is that the end effector is moving too much, want to explore fixing the pose and changing just the XYZ.

To do that, we can use the mc's set_pose_target() func instead of the set_position_target() function.

So we want to 
1. Read the current arm's pose (including both pos and rotation)
2. Tweak only the XYZ part of that pose 
3. Set the tweaked pose as the target position for the MC planner.

Possible issue:
    step 1 -> the dvrk library returns a PyKDL.Frame object reading the pose
    step 3 -> the mc library can accept 
                - Pose message
                - PoseStamped message 
                - List of 6 floats: [x, y, z, rot_x, rot_y, rot_z] 
                - List of 7 floats  [x, y, z, qx, qy, qz, qw] 

    So I need to convert a PyKDL.Frame into one of the 4 options shown above to make it MC compatible.

'''