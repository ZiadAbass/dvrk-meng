import numpy as np

p1 = [0.16480233975290323, -0.08713767642252303, 0.043108680782781454, 0.3833254045012679, 0.10491328216307251, -0.06873812566123282]
p2 = [0.13184187180232257, -0.06971014113801842, 0.03448694462622516, 0.30666032360101425, 0.08393062573045801, -0.05499050052898625]
p3 = [0.09888140385174193, -0.05228260585351382, 0.02586520846966887, 0.22999524270076072, 0.0629479692978435, -0.04124287539673969]
p4 = [0.06592093590116128, -0.03485507056900921, 0.01724347231311258, 0.15333016180050713, 0.041965312865229004, -0.027495250264493124]
p5 = [0.03296046795058064, -0.017427535284504605, 0.00862173615655629, 0.07666508090025356, 0.020982656432614502, -0.013747625132246562]

# list of joint angle lists
points = [p1,p2,p3,p4,p5]

# interpolate() takes in a list of joint configurations and a goal total number of joint configurations. 
# It interpolates to find additional configurations between each of the existing ones such that the total becomes what is required.
# Requires: `points`        -> collection of original joint angle lists
# Requires: `total_points`  -> number of joint angle lists we want in the end
def interpolate(points, total_points):
    # if the number of joint angles in the input is the same as the required number, then there's no point interpolating
    if len(points) >= total_points:
        return points
    # find out how many extra interpolated angles we need between each of the existing ones inside `points` such that the total becomes what we need
    points_per_interval = int(total_points / (len(points)-1))
    
    # will get populated with the new extended list of configurations BUT in a different format 
    # i.e. first element will contain ALL joint angles for the first joint, second element ALL the joint angles for the seconf joint, etc.
    interpolated_angles = [[],[],[],[],[],[]]
    # for each joint
    for current_joint in range(0,6):
        # all the angles for one specific joint, re-initialised for each new joint.
        temp_joint_angles = []
        # loop through all its occurances in the list of configurations
        for cfg_idx in range(0,len(points)-1):
            # interpolate points between this joint's value and the same joint's value in the next configuration.
            interpolated_joint = np.linspace(points[cfg_idx][current_joint],points[cfg_idx+1][current_joint],points_per_interval)
            temp_joint_angles.extend(interpolated_joint)
        # update the interpolated_angles with the whole list of angles for that joint and move on
        interpolated_angles[current_joint] = temp_joint_angles

    # convert the new format containing one item per joint into a format that has one item per configuration (long list of 6 joint angles)
    extended_points = []
    config = [[],[],[],[],[],[]]
    for xx in range(0,len(interpolated_angles[0])):
        config = [[],[],[],[],[],[]]
        for ii in range(0,6):
            config[ii] = interpolated_angles[ii][xx]
        extended_points.append(config)

    return extended_points
        

extended_points = interpolate(points,300)



# ----------------------------------------------------
# ----------------------------------------------------
# ----------------------------------------------------
# ----------------------------------------------------

# get_trajectories() takes in a list of goal coordinates, generates a trajectory 
# between each point and the next one, and creates a list of trajectories with all of them together.
# `extend_final` is an optional parameter that adds more points to generate in the final segment of the stroke.
# This helps the arm reach the end of the stroke accurately rather than finish early
def get_trajectories_list_from_coords(reachy, goal_coords, predict=False, points=100, extend_final=0, initial_guess=[]):
    angles_dicts = []
    repeat_final = False
    print('Generating', len(goal_coords)-1, 'trajectories for this stroke')
    for i in range(0, len(goal_coords)-1):
        # print('Generating trajectory', i, 'out of', len(goal_coords)-1)
        # add more points to the final segment of the stroke to enforce the arm going to it
        if i == (len(goal_coords)-3):
            # print('Adding more points to pre-final')
            points += int(extend_final/2)
        if i == (len(goal_coords)-2):
            # print('Adding more points to final')
            points += int(extend_final)
            repeat_final = True         # tells the generate_traj_angles() to add the final coordinate multiple times to the traj. This is to help the arm reach the end of the stroke.
        H, err = generate_traj_angles(reachy, goal_coords[i], goal_coords[i+1], points, predict=predict, repeat_final=repeat_final, initial_guess=initial_guess)
        if err:
            print('Could not obtain traj angles for points', goal_coords[i], goal_coords[i+1])
            return [], True
        angles_dicts.append(H)
    # print('Generated ', len(angles_dicts) ,'trajectories.')
    return angles_dicts, False


# get_focus_coords() takes in a list of coords that make up a stroke.
# Also takes in the number of focus points needed (i.e. points that will have IK run on them)
# and returns a list of 3D coords. Their count = focus coords argument.
def get_focus_coords(stroke, focus_points_in_stroke):
    if len(stroke) < focus_points_in_stroke:
        # print('Focus points are more than the number of coordinates in the stroke!\n Will use all points on the stroke as focus points')
        if len(stroke) > 2:
            focus_points_in_stroke = len(stroke)
        else:
            focus_points_in_stroke = 2      # manually set 2 focus points if the stroke consists of 2 coords to avoid dividing by zero below
    focus_coords = []
    # sample some points from the coordinates on the stroke
    inds = np.linspace(0, len(stroke)-1, focus_points_in_stroke)
    for ind in inds:
        focus_coords.append(stroke[int(ind)])

    return focus_coords

# curve_to_traj() takes in a stroke (list of coordinates) and turns it into one trajectory.
# It also takes in the number of focus points (i.e. points that will have IK run on them)
# Also takes in the total number of points to generate for the stroke
def curve_to_traj(reachy, stroke, focus_points_in_stroke, total_points, predict=False, extend_final=0, use_gripper=True, check_bounds=False, initial_guess=[]):
    # calculate the focus coordinates
    focus_coords = get_focus_coords(stroke, focus_points_in_stroke)

    # an empty focus_coords list indicates there was an problem with get_focus_coords()
    if len(focus_coords) == 0:
        return [], True   # return an empty trajectory to indicate that the points were invalid

    # number of points for each interval to generate for the trajectories
    points_per_interval = int(total_points / (len(focus_coords)-1))

    # now we have the condensed list of coords
    # call get_trajectories_list_from_coords(reachy, goal_coords, predict=False) to get list of dictionaries
    angles_dicts, err = get_trajectories_list_from_coords(reachy, focus_coords, predict=predict, points=points_per_interval, extend_final=extend_final, initial_guess=initial_guess)
    if err:
        print("ERROR: Failed to get trajectory from coords:", focus_coords)
        return [], True

    angles_by_motor = [[], [], [], [], [], [], [], []]
    # Look through the list and generate one traj from all of them
    for traj in angles_dicts:
        for i, motor_angles in enumerate(traj):
            angles_by_motor[i].extend(traj[motor_angles])

    # create the dictionary to send to the trajectory player
    one_big_traj = {'left_arm.shoulder_pitch': np.array(angles_by_motor[0]),
      'left_arm.shoulder_roll': np.array(angles_by_motor[1]),
      'left_arm.arm_yaw': np.array(angles_by_motor[2]),
       'left_arm.elbow_pitch': np.array(angles_by_motor[3]),
        'left_arm.hand.forearm_yaw': np.array(angles_by_motor[4]),
        'left_arm.hand.wrist_pitch': np.array(angles_by_motor[5]),
        'left_arm.hand.wrist_roll': np.array(angles_by_motor[6]),
        'left_arm.hand.gripper': np.array(angles_by_motor[7])}

    # if we are not using a gripper then delete the last entry from the dictionary.
    if not use_gripper:
        one_big_traj = remove_gripper_from_dict(one_big_traj)

    return one_big_traj, False
















