'''
Created by Ziad Abass

Purpose:
Implement a single exploration step which involves:
    1. Home the PSM
    2. Generate nearby exploration points 
    3. Have the arm explore these 4 coordinates around it (+/- dX and +/- dY) in turn
'''

import master as mm

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

'''
find_exploration_points() uses the PSM's current position and finds the 4 surrounding coordinates
an 'increment' mm away from the current position (in +/- X and +/- Y)
'''
def find_exploration_points(psm,group,increment):
    # find the arm's position in 3D
    cur_XYZ = mm.read_display_dvrk_pos(psm, verbose=False)
    # convert the increment given from mm to meters
    inc_m = float(increment)/1000

    # +/- in Y
    up_y = [cur_XYZ[0],cur_XYZ[1]+inc_m,cur_XYZ[2]]
    down_y = [cur_XYZ[0],cur_XYZ[1]-inc_m,cur_XYZ[2]]
    # -/+ in X
    down_x = [cur_XYZ[0]-inc_m,cur_XYZ[1],cur_XYZ[2]]
    up_x = [cur_XYZ[0]+inc_m,cur_XYZ[1],cur_XYZ[2]]
    
    exploration_coords = [up_y, down_y, down_x, up_x]
    return exploration_coords


if __name__ == '__main__':
    # initialise and home
    psm,group,_ = mm.init_and_home()

    # find the coordinates of the 4 exploration points
    exp_coords = find_exploration_points(psm,group,increment=15)

    # start reading the goal+ actual positions in the background
    logger = start_recording(psm)

    # go to all 4 with a fixed vertical orientation
    for idx, target_coord in enumerate(exp_coords):
        buf = "Press Enter to move to coordinate number %d" % (idx)
        raw_input(buf)
        mm.goto_xyz(psm,target_coord,6000,group,fixed_orientation='vertical')

    # stop the position recording an save the data collected
    stop_recording(logger, plot=True)
        
    psm.shutdown()
