'''
First trial to plan and execute a series of trajectories through a python script that uses both the dvrk and the moveit_commander libraries. 
'''
'''
TODO
- Find how to control speed through dvrk package
- Turn 'interpolate' off in dvrk package
- How will the acceleration/velocity lists fit in?
- Interpolate to add more waypoints
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

### Initialisations
# init dvrk objject
p = dvrk.psm('PSM1')
# init MC objects
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "psm_arm"
group = moveit_commander.MoveGroupCommander(group_name)

### initialise a ROS node
# rospy.init_node('hamada',anonymous=True)

### move robot to home position
raw_input('Initialisation completed. Press ENTER to home the PSM')
p.home()
time.sleep(1)

### obtain current joint angles from dvrk
curr_joint_pos = p.get_current_joint_position()
print('Current joint position:', curr_joint_pos)
# e.g. array([ 0.  ,  0.  ,  0.12,  0.  ,  0.  ,  0.  ])

### set current angles as starting point in mc space
# initialise a RobotState object
start_state = RobotState()
# define the joint names
start_state.joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint'] 
# define the required start position angles
start_state.joint_state.position = [0, 0, 0.2, 0, 0, 0]
# start_state.is_diff = True
group.set_start_state(start_state)

### set a goal position OR goal joint angles in mc space
group.set_named_target('zrandom')

### plan a traj using mc
traj_plan = group.plan()

### convert the plan into a list of joint angles
# extract the list of `point` objects. Each contains a list of 6 positions, velocities, accelerations, efforts and a duration
traj_points = traj_plan.joint_trajectory.points

# now only extract the positions from each traj point
pure_positions = []
for point in traj_points:
    pure_positions.append(point.positions)
# pure_positions is now a list. Each item in the list is a list of 6 motor angles.

### interpolate the list to have many more waypoints across the trajectory
# TODO STILL

### use the dvrk library to follow the list of joint angles and perform the traj
for pos in pure_positions:
    p.move_joint(np.array(pos), interpolate = True)

raw_input('Press enter to shutdown')
### Stop providing power to the arm
p.shutdown()