'''
Purpose:
Moveit_commander has a set_position_target() function that takes in an XYZ coordinate and sets it as the goal position. 
I want to test this out to make sure that the XYZ perspective of moveit matches that of the dvrk library.
    o	Send PSM anywhere
    o	Read current 3D pos through dvrk
    o	Make that same 3D position the target through moveit (set_position_target)
    o	Plan a traj from current position to the target with mc.
    o	See if the robot moves.
    o	Then plan one with a target a few cm away from current position.
    o	Plan with mc and execute with dvrk.
'''

# import master
import master as mm
import time

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

    raw_input('Press ENTER to go to the desired XYZ coord')
    # Make that same 3D position the target through moveit and go to it using dvrk
    mm.goto_xyz(psm,cp,5000,group)

    # read 3D pos from dvrk
    raw_input('Press ENTER to read current position')
    mm.read_display_dvrk_pos(psm,verbose=True)

    # See if the robot moves.
    # Then plan one with a target a few cm away from current position.
    close_pos = cp
    close_pos[0] += 0.05
    raw_input('Press ENTER to move PSM slightly in X axis')
    mm.goto_xyz(psm,close_pos,5000,group)

    # read 3D pos from dvrk
    raw_input('Press ENTER to read current position')
    mm.read_display_dvrk_pos(psm,verbose=True)

    raw_input('Press enter to shutdown')
    ### Stop providing power to the arm
    psm.shutdown()