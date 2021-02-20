import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('hamada',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "psm_arm"
group = moveit_commander.MoveGroupCommander(group_name)

print('joint_goal', group.get_current_joint_values())
'''
group.set_named_target('zhome')
group.plan()
group.go()

group.set_joint_value_target([0,0,0])
'''

# ------------------





