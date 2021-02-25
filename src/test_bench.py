import dvrk
import numpy as np
import PyKDL

# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')

# Provide power to the arm and home it.
raw_input('Press enter to home')
p.home()

# read_display_dvrk_angles() reads and returns the PSM's current joint states 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_angles(p,verbose=False):
    curr_joint_pos = p.get_current_joint_position()
    if verbose:
        print "\n=-=-=-=-=-=-=\nCurrent Joint Angles:\n", np.round(np.array(curr_joint_pos),4),"\n=-=-=-=-=-=-=\n"
    return curr_joint_pos

# read_display_dvrk_rot() reads and returns the PSM's current rotation 
# from the dvrk library's perspective. Can optionally display it too if verbose is true.
def read_display_dvrk_rot(p,verbose=False):
    curr = p.get_current_position()
    rotation = curr.M
    if verbose:
        print "\n=-=-=-=-=-=-=\n1Current Rotation:\n", rotation,"\n=-=-=-=-=-=-=\n"
    return curr.M

read_display_dvrk_rot(p,True)

# Stop providing power to the arm
p.shutdown()