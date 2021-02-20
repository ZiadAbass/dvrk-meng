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

robot = moveit_commander.RobotCommander(robot_description="/dvrk/PSM1/robot_description")



# ------------------





