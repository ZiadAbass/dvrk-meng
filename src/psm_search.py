'''
Created by Ziad Abass

Abbreviations:
OPSM -> Operating PSM, the PSM that is controlled by the surgeon which is performing the resection.
UPSM -> Ultrasound PSM, the PSM carrying the US probe being controlled by this system.

Script Purpose:
Takes in a fake 2D location and Z axis rotation for the OPSM end effector.
Uses the `bounding_rectangle` script to generate a rectangle and a search path within. 
This search path is then fed to the UPSM and it should explore that box by following
that path in a single smooth trajectory.

Steps taken
    1. Init and home the PSM
    2. Read the PSM's 3D coordinate
    3. Simulate a fake OPSM pose 
    4. Provide that to bounding_rectangle and retrieve a search path
    5. Go to the first point on that path
    6. Generate a trajectory from the retrieved path
    7. Smooth the generated trajectory
    8. Execute the smoothed trajectory
'''

import master as mm
import bounding_rectangle as br
import PyKDL
import time
import cubic_smoothing as cs
import numpy as np
import pickle


# define number of columns we want to search through
search_cols = 5

# =========== THREAD FOR RECORDING THE PSM'S POSITION ===============
# inits for recording arm's position
from threading import Thread
import time
import numpy as np
recorder = True
log_data = []
# define the filename without directories or extensions
recording_filename = "pos_rec_demo2"
# use the filename to get the full path
save_recording_filepath = './files/'+recording_filename+'.npy'
# position_updater() runs in the background and collects the PSM's position while in operation.
def position_updater(p):
    global log_data
    while recorder:
        # find goal and real motor angles
        curr_xyz = p.get_current_position().p
        # append to the recorder to plot the results later
        log_data.append(curr_xyz)
        time.sleep(0.01)
# start recording the PSM's XYZ coords
def start_recording(psm):
    # start reading the goal+ actual positions in the background
    logger = Thread(target=position_updater, args=[psm])
    logger.start()
    print('Started recording...')
    time.sleep(0.1)
    return logger
# stop recording the PSM's XYZ coords. 
# Set plot to True to directly plot the recording into a PNG image with the same filename
def stop_recording(logger, plot=False):
    global recorder
    # stop the position recording an save the data collected
    recorder = False
    logger.join()
    np.save(save_recording_filepath, log_data)
    print('Stopped recording, saved file.')
    if plot:
        print("Will plot the recording...")
        import read_recording as rr
        rr.main(recording_filename, plot_title="")
# =====================================================================
# =====================================================================

if __name__ == '__main__':
    # - init and home
    psm,group,_ = mm.init_and_home()

    # - read psm 3D coordinate
    xyz = mm.read_display_dvrk_pos(psm,verbose=False)

    # - simulate OPSM pose, assume it is a few cm away and rotated 40 degrees
    OPSM_sim_xy = [xyz[0]+0.03, xyz[1]+0.02]
    OPSM_sim_rot = 40

    # - provide that to bounding_rectangle and retrieve waypoints for a search path
    _, path = br.main(rec_width_in=0.10, rec_height_in=0.05, gripper_location=OPSM_sim_xy, gripper_rotation_degrees=OPSM_sim_rot, gripper_displacement=0.3, search_columns=search_cols, height_padding=0.2, plot=True)

    # - go to the first point on that path
    path_beginning = [path[0][0],path[0][1],xyz[2]]
    raw_input("press enter to go to the path's beginning")
    mm.goto_xyz(psm, goal_coords=path_beginning, total_points=3000,group=group,fixed_orientation='vertical')

    # convert 2D XY coords to 3D
    path_3D = []
    for step in path:
        # convert 2D into 3D using a fixed Z (height)
        goal = [step[0],step[1],xyz[2]]
        path_3D.append(goal)
    
    #  -------------------------------------------
    
    # - generate a single trajectory that goes through all the coordinates in the retrieved path
    traj = mm.generate_traj(psm, goal_coords_list=path_3D, total_points=20000, group=group,fixed_orientation='vertical')

    # start reading the goal + actual USPSM positions in the background
    logger = start_recording(psm)

    # smooth and execute the traj
    raw_input("Press enter to follow the traj")
    mm.smooth_and_play(p=psm, traj=traj, smooth_factor=20)
    time.sleep(0.2)
    
    # stop the position recording an save the data collected
    stop_recording(logger, plot=True)

    # close
    time.sleep(1)
    psm.shutdown()