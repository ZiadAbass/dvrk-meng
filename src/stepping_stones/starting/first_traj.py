'''
First trial to plan and execute a series of trajectories through a python script that uses both the dvrk and the moveit_commander libraries. 
'''
'''
TODO
- Find how to control speed through dvrk package
- How will the acceleration/velocity lists fit in?
'''

# import both dvrk and moveit_commander (MC)
# init both dvrk and MC objects
# obtain current joint angles from dvrk
# set current angles as starting point in mc space
# set a goal position OR goal joint angles in mc space
# plan a traj using mc
# convert the plan into a list of joint angles
# interpolate the list to have many more waypoints across the trajectory
# use the dvrk library to follow the list of joint angles and perform the traj


# import both dvrk and moveit_commander (MC)
import dvrk

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState 
from sensor_msgs.msg import JointState 

import sys
import copy
import rospy
import numpy as np
import time
# import PyKDL

# create a RobotState template instead of initialising a new one each time
# initialise a RobotState object
start_state = RobotState()
# define the joint names
start_state.joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint'] 

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
    return p,group

# set_mc_start_state() uses the RobotState template to set the start position of the robot from the 
# move_commande's perspective. It always sets the starting point of the moveit_commander to the current position of the simulated robot in the dvrk library.
# takes in the dvrk PSM object
def update_mc_start_state(p):
    # obtain current joint angles from dvrk (list of 6 angles)
    curr_joint_pos = p.get_current_joint_position()
    # print('Current joint position:', curr_joint_pos)
    # define the required start position angles
    start_state.joint_state.position = curr_joint_pos
    # start_state.is_diff = True
    group.set_start_state(start_state)

# set_named_target_and_plan() takes in the name of a saved configuration and plans its path,
# returning the RobotTrajectory object
def set_named_target_and_plan(target_name):
    # set a goal position
    group.set_named_target(target_name)
    ### plan a traj using mc
    traj_plan = group.plan()
    return traj_plan

# set_joint_target_and_plan() takes in a list of 6 angle values represnting a configuration and plans its path,
# returning the RobotTrajectory object
def set_joint_target_and_plan(joint_config):
    # set a goal configuration
    print('setting', joint_config)
    group.set_joint_value_target(np.array(joint_config))
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
def goto_named_config(p,target_name,total_points):
    ### set current angles as starting point in mc space
    update_mc_start_state(p)

    ### set a goal position
    traj_plan = set_named_target_and_plan('zrandom')

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, 7000)

    raw_input('Ready to execute the trajectory. Press Enter.')
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)

# goto_named_config() plans and executes a trajectory towards a given angle configuration
# also takes in `total_points` total number of points to include in the final trajectory after interpolation
def goto_angle_config(p,target_angles,total_points):
    ### set current angles as starting point in mc space
    update_mc_start_state(p)

    ### set a goal position
    traj_plan = set_joint_target_and_plan(target_angles)

    # take only the position angles from the plan
    pure_positions = extract_pos_from_plan(traj_plan)

    # interpolate to obtain many intermediate points
    extended_points = interpolate(pure_positions, 7000)

    raw_input('Ready to execute the trajectory. Press Enter.')
    ### use the dvrk library to follow the list of joint angles and perform the traj
    for pos in extended_points:
        p.move_joint(np.array(pos), interpolate = False)


if __name__ == '__main__':
    # initialise the required objects
    p,group = init_dvrk_mc()

    ### move robot to home position
    raw_input('Initialisation completed. Press ENTER to home the PSM')
    p.home()
    time.sleep(1)

    # ------------- # GO TO ZRANDOM # ------------- #
    goto_named_config(p,'zrandom', 7000)

    # ------------- # GO TO ZHOME # ------------- #
    # goto_named_config(p,'zhome', 7000)

    # ------------- # GO TO ANGLE # ------------- #
    new_goal = [0,0,0,0,0,0]
    goto_angle_config(p,new_goal,7000)

    # ------------- # ---------- # ------------- #

    raw_input('Press enter to shutdown')
    ### Stop providing power to the arm
    p.shutdown()









