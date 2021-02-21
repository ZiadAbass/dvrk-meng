import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState 
from sensor_msgs.msg import JointState 

rospy.init_node('hamada',anonymous=True)

# init MC objects
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "psm_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# print('CURRENT:', group.get_current_joint_values())

'''
# create a joint state object
joint_state = JointState() 
# set the joint state parameters
joint_state.header = Header() 
joint_state.header.stamp = rospy.Time.now() 
# joint_state.name = ['psm_rev_joint', 'psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint']
joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint']
joint_state.position = [0, 0, 0.2, 0, 0, 0] 

# create a robot state object that will include the joint state object
moveit_robot_state = RobotState() 
moveit_robot_state.joint_state = joint_state 
# set the start state
group.set_start_state(moveit_robot_state)
'''

start_state = RobotState()
start_state.joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint']
start_state.joint_state.position = [0, 0, 0.2, 0, 0, 0]
start_state.is_diff = True
group.set_start_state(start_state)

# set names target
group.set_named_target('zrandom')
plann = group.plan()

# extract the list of `point` objects. Each contains a list of 6 positions, velocities, accelerations, efforts and a duration
traj_points = plann.joint_trajectory.points

# now only extract the positions from each traj point
pure_positions = []
for point in traj_points:
    pure_positions.append(point.positions)
# pure_positions is now a list. Each item in the list is a list of 6 motor angles.

print('Length:', len(pure_positions))
print('Potsitions:', pure_positions)

