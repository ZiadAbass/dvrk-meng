'''
Purpose: 
- Send arm to a specific 3D coordinate
- Have the arm explore 4 coordinates around it (+/- dX and +/- dY)
'''

import master as mm


# exclude_closest_coord takes in a target coordinate and a list of 'options' coordinates.
# From the given options, it finds the one closest to the target coordinate and removes it,
# returning a list of all the remaining options.
def exclude_closest_coord(target, options):
    # TODO: Do stuff
    return options[:-1]

# find_exploration_points() uses the PSM's current position and finds the 4 surrounding coordinates
# an 'increment' mm away from the current position (in +/- X and +/- Y)
# If `exclude_position` is supplied, then only 3 exploration points are returned. 
#   Of the 4 generate coords, the one closest to the exclude_position is removed.
def find_exploration_points(psm,group,increment, exclude_position=False):
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
    
    if exclude_position == False:
        exploration_coords = [up_y, down_y, down_x, up_x]
    else:
        # Of down_y, up_y, down_x and up_x, find the closest one to provided coordinate
        exploration_coords = exclude_closest_coord(target=exclude_position, options=[up_y, down_y, down_x, up_x])

    return exploration_coords

# check_vessel_location() simulates sending an US slice to the CVML block and receiving
# the location of the relevant blood vessel.
# Right now, it takes in the simulated result which we want to return.
# Returns False if the blood vessel cannot be found
# Returns True if the blood vessel is on the vertical centerline
# Otherwise, returns the distance of the vessel's center from the vertical centerline of the slice.
def get_vessel_location(sim_result):
    # TODO: Do stuff
    return sim_result

# save_slice() saves the ultrasound slice at the current location as well as
# the current location of the PSM.
def save_slice():
    psm_position = mm.read_display_dvrk_pos(psm,verbose=False)
    print "Ultrasound slice saved, arm is at", psm_position

# explore() loops through a given set of exploration coords.
# at each coords, it checks to see wherher or not the probe moved closer to the vessel.
# returns True if the probe moved closer to the vessel.
# returns False if none of the given coords moved the probe closer to the vessel.
def explore(psm, exp_coords):
    # go to all exploration points with a fixed vertical orientation
    for idx, target_coord in enumerate(exp_coords):
        buf = "Press Enter to move to coordinate number %d" % (idx)
        raw_input(buf)
        mm.goto_xyz(psm,target_coord,6000,group,fixed_orientation='vertical')
        # check where the vessel is in the US slice
        dv = get_vessel_location(12)
        # if the vessel has gone out of the FOV or has moved further away
        if (dv == False) or (dv > dev_Q):
            # leave if we have looped through all the coords given
            if idx == len(pp)-1:
                # return False to indicate none of the exploration coords got the probe closer to the vessel.
                print "Looped through", len(exp_coords), "exploration coords. None of them got the probe closer to the vessel"
                return False
            # otherwise go to the next coord
            print "Probe has moved further away from the vessel. Continueing to next exploration coord..."
            continue
        # if the vessel is on the centerline
        elif dv == True:
            save_slice()
        # we have now moved closer to the vessel, so no need to 
        # continue looping through the current exploration coords, 
        # we should generate 3 new ones. 
        # Return true to indicate a successful exploration
        return True

def start_sequence(psm,group):

    # check whether or not the vessel is in the field of view
    dv = get_vessel_location(15)
    
    # if the vessel was not detected
    if dv == False:
        print "Vessel is outside the probe's FOV, should continue searching"
        while dv != True:
            # TODO: Proceed with a search algo that makes PSM scan the are
            # after each search step, check for the vessel again
            dv = get_vessel_location(False)
            print "Still searching..."
    
    # if the vessel is on the vertical centerline
    if dv == True:
        print "Vessel is on the centerline, saving the slice"
        save_slice()
    else:
        # save the vessel's current deviation from the centerline
        dev_Q = get_vessel_location()
    
    # update pos_Q with the PSM's current position before sending it to exploration coords
    pos_Q = mm.read_display_dvrk_pos(psm,verbose=False)
    
    #  ---------------
    # TODO: This loop currently happens 10 times.
    # It should be a while loop using a flag that indicates when it's time to stop
    for iters in range(0,10):
        if iters == 0:
            # if this is the first exploration journey, 
            # find the coordinates of the 4 exploration points
            exp_coords = find_exploration_points(psm,group,increment=15)
        else:
            # now we have moved closer to the vessel, we only want to find 3 exploration
            # coordinates, the fourth would bring us back to where we were before.
            exp_coords = find_exploration_points(psm,group,increment=15, exclude_position=pos_Q)

            # update pos_Q with the PSM's current position before sending it to more exploration coords
            pos_Q = mm.read_display_dvrk_pos(psm,verbose=False)
            # save the vessel's current deviation from the centerline
            dev_Q = get_vessel_location()

        # visit the exploration coords and (hopefully) stop at one that gets the probe closer to the vessel
        exp_result = explore(psm, exp_coords)

        # if the exploration was successful
        if exp_result:
            print "Successful exploration, moved in the right direction"
        else:
            print "Unsuccessful exploration, none of the explored coords moved the probe in the right direction"
            # TODO ?



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