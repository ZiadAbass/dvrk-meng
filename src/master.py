'''
Created by Ziad Abass
Serves as an internal API for me to interface with both the dvrk and moveit_commander libraries together
'''

# import both dvrk and moveit_commander (MC)
import dvrk

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState 
from sensor_msgs.msg import JointState 

import cubic_smoothing as cs    # TODO: Document this <-

import sys
import copy
import rospy
import numpy as np
import time

''' TODO's
- find out frequency of arm to convert number of traj interpolated points to duration of traj
- cubic smoothing for the traj
- introduce check_bounds function for safety
- change interpolate() when it's called form goto_list_coords to accomodate the fact that 
    cordinates given are not equally spaced. Solution is to use the XYZ coords supplied to 
    find a ratio of all the distances between consequetive points. The total number of interpolation
    points is then divided according to these ratios instead of being divided equally.
'''

# create a RobotState template instead of initialising a new one each time
# initialise a RobotState object
start_state = RobotState()
# define the joint names
start_state.joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint'] 
# define the qx,qy,qz,qw leading to a vertical end effector orientation
vertical_orientation = [-0.7088969395364019, 0.29921509795680007, -0.29572446227672855, 0.5661117351563785]

### Initialisations
# init_dvrk_mc() intiialises the required objects for both the dvrk and the move_commander libraries
def init_dvrk_mc():
    # init dvrk objject
    p = dvrk.psm('PSM1')
    # init MC objects
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "psm_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    return p,group,scene

# init_and_home() initialises both the dvrk and moveit_commander objects, then homes the
# dvrk PSM to activate it and prepare it for any commands
def init_and_home():
    p,group,scene = init_dvrk_mc()
    print "Initialisation complete, will home the PSM"
    p.home()
    time.sleep(0.1)
    return p,group,scene

# init_dvrk_home() initialises only the dvrk object, then homes the
# dvrk PSM to activate it and prepare it for any commands
def init_dvrk_home():
    # init dvrk objject
    p = dvrk.psm('PSM1')
    print "Initialisation complete, will home the PSM"
    p.home()
    time.sleep(0.25)
    return p

# set_mc_start_state() uses the RobotState template to set the start position of the robot from the 
# move_commande's perspective. 
# Unless a specific start positon (6 angles) is supplied, it sets the starting point of the 
# moveit_commander to the current position of the simulated robot in the dvrk library.
# takes in the dvrk dvrk and MC objects.
def update_mc_start_state(p,group, required_start=[]):
    # if no specific start position is given
    if required_start == []:
        # obtain current joint angles from dvrk (list of 6 angles)
        curr_joint_pos = p.get_current_joint_position()
    # if 6 initial angles are supplied then use them
    elif len(required_start) == 6:
        curr_joint_pos = required_start
    else:
        print "Invalid 'required_start' angles supplied. They should be 6 angles. Given:", required_start, "Exiting..."
        return True
    # define the required start position angles
    start_state.joint_state.position = curr_joint_pos
    # start_state.is_diff = True
    group.set_start_state(start_state)

def update_mc_start_go(p,group):
    # obtain current joint angles from dvrk (list of 6 angles)
    curr_joint_pos = p.get_current_joint_position()
    # print('Current joint position:', curr_joint_pos)
    # define the required start position angles
    start_state.joint_state.position = curr_joint_pos
    group.set_joint_value_target(start_state)
    traj_plan = group.plan()
    # raw_input("press enter to execute")
    # group.go()
    group.execute(traj_plan)
    # raw_input("press enter to set start state")
    group.set_start_state(start_state)

# set_named_target_and_plan() takes in the name of a saved configuration and plans its path,
# returning the RobotTrajectory object
def set_named_target_and_plan(target_name,group):
    # set a goal position
    group.set_named_target(target_name)
    ### plan a traj using mc
    traj_plan = group.plan()
    return traj_plan

# set_joint_target_and_plan() takes in a list of 6 angle values represnting a configuration and plans its path,
# returning the RobotTrajectory object
def set_joint_target_and_plan(joint_config, group):
    # set a goal configuration
    print('setting', joint_config)
    group.set_joint_value_target(np.array(joint_config))
    ### plan a traj using mc
    traj_plan = group.plan()
    return traj_plan

# set_XYZ_target_and_plan() takes in a set of XYZ coords representing a goal position and plans its path,
# returning the RobotTrajectory object
def set_XYZ_target_and_plan(goal_coords, group):
    # set a goal configuration
    print "Setting XYZ:", goal_coords
    group.set_position_target(goal_coords)
    ### plan a traj using mc
    traj_plan = group.plan()
    return traj_plan

# set_pose_target_and_plan() takes in a pose representing a goal position and plans its path,
# returning the RobotTrajectory object
def set_pose_target_and_plan(goal_pose, group):
    # set a goal configuration
    print "Setting Pose:", goal_pose
    group.set_pose_target(goal_pose)
    ### plan a traj using mc
    traj_plan = group.plan()
    return traj_plan

# extract_pos_from_plan() takes in the RobotTrajectory object and extracts the list of positional angles from it 
def extract_pos_from_plan(traj_plan):
    ### convert the plan into a list of joint angles
    # extract the list of `point` objects. Each contains a list of 6 positions, velocities, accelerations, efforts and a duration
    traj_points = traj_plan.joint_trajectory.points

    # now only extract the positions from each traj point
    pure_positions = []
    for point in traj_points:
        pure_positions.append(point.positions)
    # pure_positions is now a list. Each item in the list is a list of 6 motor angles.
    return pure_positions


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

# goto_named_config() plans and executes a trajectory towards a given named configuration
# also takes in `total_points` total number of points to include in the final trajectory after interpolation
def goto_named_config(p,target_name,total_points,group):
    ### set current angles as starting point in mc space
    update_mc_start_state(p,group)

    ### set a goal position
    traj_plan = set_named_target_and_plan('zrandom', group)

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, total_points)

    raw_input('Ready to execute the trajectory. Press Enter.')
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)

# goto_angle_config() plans and executes a trajectory towards a given angle configuration
# also takes in `total_points` total number of points to include in the final trajectory after interpolation
def goto_angle_config(p,target_angles,total_points,group):
    ### set current angles as starting point in mc space
    update_mc_start_state(p,group)

    ### set a goal position
    traj_plan = set_joint_target_and_plan(target_angles, group)

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, total_points)

    raw_input('Ready to execute the trajectory. Press Enter.')
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)

# goto_xyz() plans and executes a trajectory towards a single given a target XYZ coordinate.
# also takes in `total_points` total number of points to include in the final trajectory after interpolation
# fixed_orientation: optional param used to define an end effector orientation. 
#   Leave out for the Ik solver to decide own orientation
#   Use 'vertical' to keep the end effector vertical when going anywhere,
#   Otherwise, provide qx,qy,qz,qw to define the orientation
def goto_xyz(p,goal_coords,total_points,group,fixed_orientation=''):
    ### set current angles as starting point in mc space
    update_mc_start_state(p, group)

    ### set a goal position
    # if no fixed orientation is provided
    if fixed_orientation == '':
        traj_plan = set_XYZ_target_and_plan(goal_coords, group)
    elif fixed_orientation == 'vertical':
        # create a target pose with the XYZ and the default vertical orientation
        target = goal_coords
        target.extend(vertical_orientation)
        traj_plan = set_pose_target_and_plan(target, group)
    elif len(fixed_orientation) == 4:
        # create a target pose with the XYZ and any provided orientation
        target = goal_coords
        target.extend(fixed_orientation)
        traj_plan = set_pose_target_and_plan(target, group)
    else:
        print('ERROR: Provide fixed_orientation is not valid, leaving.')
        return

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, total_points)

    time.sleep(0.1)
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)

# generate_traj() take in a list of XYZ coords and generates a single trajectory that passes through all of them
#   total_points: total number of points to include in the final trajectory after interpolation
#   fixed_orientation: optional param used to define an end effector orientation. 
#   Leave out for the Ik solver to decide own orientation
#   Use 'vertical' to keep the end effector vertical when going anywhere,
#   Otherwise, provide qx,qy,qz,qw to define the orientation
def generate_traj(p,goal_coords_list,total_points,group,fixed_orientation=''):
    ### set current angles as starting point in mc space
    update_mc_start_state(p, group)

    traj_waypoints = []
    for goal_coords in goal_coords_list:
        # check coordinate is valid
        if len(goal_coords) != 3:
            print "Invalid 3D coordinate given:", goal_coords, ". Will not perform trajectory."
            return True
        # TODO: Check if XYZ is within bounds
        # reset pure_positions
        pure_positions = []
        # set a goal position
        #   if no fixed orientation is provided
        if fixed_orientation == '':
            traj_plan = set_XYZ_target_and_plan(goal_coords, group)
        elif fixed_orientation == 'vertical':
            # create a target pose with the XYZ and the default vertical orientation
            target = goal_coords
            target.extend(vertical_orientation)
            traj_plan = set_pose_target_and_plan(target, group)
        elif len(fixed_orientation) == 4:
            # create a target pose with the XYZ and any provided orientation
            target = goal_coords
            target.extend(fixed_orientation)
            traj_plan = set_pose_target_and_plan(target, group)
        else:
            print('ERROR: Provide fixed_orientation is not valid, leaving.')
            return

        # take only the position angles from the plan
        pure_positions = extract_pos_from_plan(traj_plan)

        print("pure_positions length is", len(pure_positions))

        # collect all the waypoint angles needed to visit all the XYZ coords
        traj_waypoints.extend(pure_positions)

        # set end of this path as the starting point in mc space for the next point
        update_mc_start_state(p, group, required_start=pure_positions[-1])

    # interpolate to obtain many intermediate points along the whole trajectory
    # assumes that the points are evenly spaced
    full_traj_points = interpolate(traj_waypoints, total_points)
    
    return full_traj_points

# smooth_and_play() takes in a trajectory (list of lists of 6 angles) and smooths it using
# cubic/spline smoothing. It then has the dvrk simulated PSM execute the trajectory
# smooth_factor defines the number of key points used for spline smoothing.
def smooth_and_play(p, traj, smooth_factor=5):
    # smooth the trajectory
    smoothed_traj = cs.cubic_smooth(traj=np.array(traj), kp_count=smooth_factor)
    time.sleep(0.1)
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in smoothed_traj:
        p.move_joint(np.array(pos), interpolate = False)


# goto_pose() plans and executes a trajectory towards a given a target pose (includes both XYZ and orientation).
# also takes in `total_points` total number of points to include in the final trajectory after interpolation
def goto_pose(p,goal_pose,total_points,group):
    ### set current angles as starting point in mc space
    update_mc_start_state(p, group)

    ### set a goal position
    traj_plan = set_pose_target_and_plan(goal_pose, group)

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, total_points)

    time.sleep(0.1)
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)

# read_display_mc_pos() reads and returns the moveit_commander's current XYZ coordinates.
# from the MC library's perspective. Can optionally display it too if verbose is true.
def read_display_mc_pos(g,verbose=False):
    mc_pose = g.get_current_pose().pose.position
    # invert X and Y as the positive direction in MC space is the oppostie of that in the dvrk space
    curr_pos.y = mc_pose.y*-1
    curr_pos.x = mc_pose.x*-1
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent MC XYZ position:", np.round(mc_pose.x,4), np.round(mc_pose.y,4), np.round(mc_pose.z,4),"\n=-=-=-=-=-=-=\n"
    return mc_pose

# read_display_dvrk_pos() reads and returns the PSM's current XYZ coordinates 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_pos(p,verbose=False):
    curr_pos = p.get_current_position()
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent XYZ position:", np.round(curr_pos.p[0],4), np.round(curr_pos.p[1],4), np.round(curr_pos.p[2],4),"\n=-=-=-=-=-=-=\n"
    return curr_pos.p

# read_display_dvrk_rot() reads and returns the PSM's current rotation 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_rot(p,verbose=False):
    curr = p.get_current_position()
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent Rotation:", curr.M,"\n=-=-=-=-=-=-=\n"
    return curr.M

# read_display_dvrk_angles() reads and returns the PSM's current joint states 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_angles(p,verbose=False):
    curr_joint_pos = p.get_current_joint_position()
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent Joint Angles:\n", np.round(np.array(curr_joint_pos),4),"\n=-=-=-=-=-=-=\n"
    return curr_joint_pos

# read_display_dvrk_pose() reads and returns the PSM's full current pose 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_pose(p,verbose=False):
    curr_pose = p.get_current_position()
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent Pose:", curr_pose,"\n=-=-=-=-=-=-=\n"
    return curr_pose

'''
if __name__ == '__main__':
    # initialise the required objects
    p,group = init_dvrk_mc()

    ### move robot to home position
    raw_input('Initialisation completed. Press ENTER to home the PSM')
    p.home()
    time.sleep(1)

    # ------------- # GO TO ZRANDOM # ------------- #
    goto_named_config(p,'zrandom', 7000,group)

    # ------------- # GO TO ZHOME # ------------- #
    # goto_named_config(p,'zhome', 7000)

    # ------------- # GO TO ANGLE # ------------- #
    new_goal = [0,0,0,0,0,0]
    goto_angle_config(p,new_goal,7000, group)

    # ------------- # ---------- # ------------- #

    raw_input('Press enter to shutdown')
    ### Stop providing power to the arm
    p.shutdown()
'''








