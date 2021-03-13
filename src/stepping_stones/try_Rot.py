'''
Purpose:
Is the arm’s orientation (AKA rotation) affected without moving the final 3 joints? 

- First Test: 
    Need to send the arm to a different 3D coordinate,  
    Check the before/after values for the final 3 joint values. 
    Check the before/after values for the rotation (.M of what get_current_position returns). 
    Result: both changed 

- Second test:
    Keep the arm at same place and change values for the final 3 joints 
    Read rotation before and after and compare. 
    Result: Rotation changed when final 3 angles (gripper) changed. 

Conclusion: Changing the final 3 joint angles changes the rotation. 
'''

# import master
import master as mm
import time
import numpy as np

# create a RobotState template instead of initialising a new one each time
# initialise a RobotState object
start_state = mm.RobotState()
# define the joint names
start_state.joint_state.name = ['psm_yaw_joint', 'psm_pitch_joint', 'psm_main_insertion_joint', 'psm_tool_roll_joint', 'psm_tool_pitch_joint', 'psm_tool_yaw_joint'] 


if __name__ == '__main__':
    # initialise the required objects
    psm,group = mm.init_dvrk_mc()

    ### move robot to home position
    raw_input('Initialisation completed. Press ENTER to home the PSM')
    psm.home()
    time.sleep(1)

    # read 3D pos from dvrk
    raw_input('Press ENTER to read current position')
    cp = mm.read_display_dvrk_pos(psm,verbose=True)
    mm.read_display_dvrk_rot(psm,verbose=True)
    mm.read_display_dvrk_angles(psm,verbose=True)
    
    # Move the final 3 joints
    raw_input('Press ENTER to move final 3 joins')
    # move all joints - relative, incremental in joint space
    psm.dmove_joint(np.array([0.0, 0.0, 0.0, -0.05, 0.4, 0.25]), interpolate = True)
    

    # read 3D pos from dvrk
    raw_input('Press ENTER to read current position')
    mm.read_display_dvrk_pos(psm,verbose=True)
    mm.read_display_dvrk_rot(psm,verbose=True)
    mm.read_display_dvrk_angles(psm,verbose=True)

    raw_input('Press enter to shutdown')
    ### Stop providing power to the arm
    psm.shutdown()