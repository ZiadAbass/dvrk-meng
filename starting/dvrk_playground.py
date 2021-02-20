
'''
Playing around with the dvrk library for the first time
'''

import dvrk
import numpy as np
import PyKDL

# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')

# Provide power to the arm and home it.
raw_input('Press enter to home')
p.home()

# /-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
# /-/-/-/-/-/-/-/-/-/ Reading from the arm /-/-/-/-/-/-/-/-/-/
# /-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
''' (from the API - arm.py)
Current vs Desired position
===========================
The arm controller can provide two different positions at any given
time.  The current position is the position measured by the sensors
(in most cases, encoders).  This position defines the physical
position of the system.  The desired joint position is the position
sent to the low level controller (e.g. `PID
<https://en.wikipedia.org/wiki/PID_controller>`_).  The desired
cartesian position is calculted using the desired joint position.
When using a `trajectory
<http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion>`_,
the desired position is not the final goal but the last set point
generated for the trajectory.
Desired positions might differ from the physical positions due to
`forces (gravity, friction, ...) <https://en.wikipedia.org/wiki/Force>`_ applied on the arm.  When
implementing an incremental move, one should always use the last
desired position.  If one needs to track the arm, it is better to
use the current position.
'''

raw_input('Press enter to retrieve current info')
# retrieve current info 
# # each call returns a list of 6 joint angles (np.array)
curr_joint_pos = p.get_current_joint_position()
curr_joint_vel = p.get_current_joint_velocity()
curr_joint_eff = p.get_current_joint_effort()
print('-----------------')
print('Current joint position:', curr_joint_pos)
print('Current joint velocity:', curr_joint_vel)
print('Current joint effort:', curr_joint_eff)

# retrieve PID desired position and effort computed
# # each call returns a list of 6 joint angles (np.array)
raw_input('Press enter to retrieve PID desired info')
des_joint_pos = p.get_desired_joint_position()
des_joint_eff = p.get_desired_joint_effort()
print('-----------------')
print('Desired joint position:', des_joint_pos)
print('Desired joint effort:', des_joint_eff)

# retrieve cartesian current and desired positions
# Return `PyKDL.Frame` objects.
# These objects consist of a 3x3 pose matrix, as well as a 1x3 origin list showing the XYZ coords
# More on PyKDL Frames -> http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
raw_input('Press enter to retrieve Cartesian info')
curr_pos = p.get_current_position()
des_pos = p.get_desired_position()
print('-----------------')
print('Current position:', curr_pos)
print('Desired position:', des_pos)


# /-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
# -/-/-/-/-/-/-/-/-/-/-/ Moving the arm /-/-/-/-/-/-/-/-/-/-/-
# /-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
''' (from the API - arm.py)
Interpolation
=============
If the `interpolation` flag is set to `True` (default), the arm
controller will use a `trajectory generator
<http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion>`_
to create set points between the current position and the position
requested by the user.  If your desired position is "far" from the
current position, you should always set the `interpolate` flag to
`True`.
The only case where you should override the default and set
`interpolate` to `False` is if you are sending positions close to each
other.  For example, when `tele-operating
<https://en.wikipedia.org/wiki/Teleoperation>`_, all the master
positions you will receive will define a continuous trajectory with
positions close to each other.
It is important to note that when `interpolate` is set to `False`,
sending a new goal that is far from the last desired position will
likely trigger a `PID tracking error <https://en.wikipedia.org/wiki/PID_controller>`_.
'''

# Moving in joint space
# `move` is absolute (SI units) (Absolute move in joint space)
# `dmove` is relative (Incremental move in joint space)

# move a single joint - absolute in joint space
# Provide the absolute amount in which you want to move index by (list)
# Provide the index of the joint to move (starts at 0) (list)
p.move_joint_one(0.2, 0, interpolate = True) # Absolute index move of 1 joint in joint space (first joint in this case)

# move a single joint - relative, incremental in joint space
# provide the incremental amount in which you want to move index by (float)
# provide the index of the joint you want to move, this is an integer
p.dmove_joint_one(-0.05, 2, interpolate = True) # Incremental index move of 1 joint in joint space (third joint in this case)

# move multiple joints - absolute in joint space
p.move_joint_some(np.array([0.0, 0.0]), np.array([0, 1]), interpolate = True)
# move multiple joints - relative, incremental in joint space
p.dmove_joint_some(np.array([-0.1, -0.1]), np.array([0, 1]), interpolate = True)

# move all joints - absolute in joint space
p.move_joint(np.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.0]), interpolate = True)
# move all joints - relative, incremental in joint space
p.dmove_joint(np.array([0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 0.0]), interpolate = True)

# move in cartesian space
# there are only 2 methods available, dmove and move
# both accept PyKDL Frame, Vector or Rotation

# Absolute translation in cartesian space.
p.move(PyKDL.Vector(0.0, 0.0, -0.05), interpolate = True)       # move such that end effector becomes at (0, 0, -0.05)
# Incremental motion in cartesian space.
p.dmove(PyKDL.Vector(0.0, 0.05, 0.0), interpolate = True)       # move by 5 cm in Y direction 

# save current orientation
old_orientation = p.get_desired_position().M
# save current XYZ position
old_position = p.get_desired_position().p

# rotate about an axis
r = PyKDL.Rotation()
r.DoRotX(math.pi * 0.25)    # rotate about the X axis
p.dmove(r)

# p.move(old_orientation)

raw_input('Press enter to shutdown')
# Stop providing power to the arm
p.shutdown()