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
    print "increment is", increment
    inc_m = float(increment)/1000
    print "inc_m is", inc_m

    # +/- in Y
    up_y = [cur_XYZ[0],cur_XYZ[1]+inc_m,cur_XYZ[2]]
    down_y = [cur_XYZ[0],cur_XYZ[1]-inc_m,cur_XYZ[2]]
    # -/+ in X
    down_x = [cur_XYZ[0]-inc_m,cur_XYZ[1],cur_XYZ[2]]
    up_x = [cur_XYZ[0]+inc_m,cur_XYZ[1],cur_XYZ[2]]
    
    exploration_coords = [up_y, down_y, down_x, up_x]

    return exploration_coords

# check_vessel_location() simulates sending an US slice to the CVML block and receiving
# the location of the relevant blood vessel.
# Right now, it takes in the simulated result which we want to return.
# Returns False if the blood vessel cannot be found
# Otherwise, returns the distance of the vessel's center from the vertical centerline of the slice.
def get_vessel_location(sim_result):
    return sim_result


def start_sequence(psm,group):
    # find the coordinates of the 4 exploration points
    exp_coords = find_exploration_points(psm,group,increment=15)
    # go to all 4 with a fixed vertical orientation
    for idx, target_coord in enumerate(exp_coords):
        buf = "Press Enter to move to coordinate number %d" % (idx)
        raw_input(buf)
        mm.goto_xyz(psm,target_coord,6000,group,fixed_orientation='vertical')
    # need to check the vessel's position at each of these 4 coords
    # as soon as you explore a point at which the vessel is closer to the centerline then stop there.
    # if 

if __name__ == '__main__':
    # initialise and home
    psm,group = mm.init_and_home()

    start_sequence(psm,group)

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