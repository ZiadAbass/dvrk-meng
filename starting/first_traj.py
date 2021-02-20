'''
First trial to plan and execute a series of trajectories through a python script that uses both the dvrk and the moveit_commander libraries. 
'''

# init both dvrk and moveit_commander (MC) objects
# obtain current joint angles from dvrk
# set current angles as starting point in mc space
# set a goal position OR goal joint angles in mc space
# plan a traj using mc
# convert the plan into a list of joint angles
# interpolate the list to have many more waypoints across the trajectory
# TODO: How will the acceleration/velocity lists fit in?
# use the dvrk library to follow the list of joint angles and perform the traj