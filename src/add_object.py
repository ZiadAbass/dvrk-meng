'''
Created by Ziad Abass

Purpose:
Demonstrate both adding obstacles to the scene programmatically and PSM collision avodiance.

Sets a goal position for the PSM and adds an obstacle to the moveit planning scene that is in the PSM's direct route to the goal position.
A trajectory is then generated, smoothed and executed to see if the PSM would collide with it.
The movement is recorded using the parallel thread method and plotted results can be found in the report.
The same is then done without adding the obstacle to make sure that the obstacle's presence was the factor making the difference. 

Steps taken:
    - add an object to the moveit planning scene environment
    - check the object was actually added through the GUI
    - give the PSM a goal location that would normally have the PSM collide with the object on the way
    - have the PSM follow that trajectory and record its position while it does
    - plot the recorded path of the PSM afterwards
    - exclude the code that adds the obstacle and send the arm to the same goal location
    - plot the recorded path and check whether or not having the obstacle present had an impact on the generate trajectory.
'''

import master as mm
import time
import numpy as np
import geometry_msgs.msg

# =========== THREAD FOR RECORDING THE PSM'S POSITION ===============
# inits for recording arm's position
from threading import Thread
recorder = True
log_data = []
# define the filename without directories or extensions
recording_filename = "pos_rec_obstacle_new"
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
add_sphere() adds a spherical shape to moveit's planning scene
Arguments
    - scene: the initialised moveit PlanningSceneInterface()
    - frame_id: the ID of the existing PSM link relative to which the position of the added object is defined
    - object_name: a unique name for the object added to the scene
'''
def add_sphere(scene, frame_id, object_name):
    sphere_pose = geometry_msgs.msg.PoseStamped()
    sphere_pose.header.frame_id = frame_id
    
    # set pre-defined XYZ relative location for the object ----------
    x_in = -0.05    #float(raw_input("enter required X"))
    y_in = 0        #float(raw_input("enter required Y"))
    z_in = -0.05    #float(raw_input("enter required Z"))

    # OR ask user to define XYZ relative location for the object ----------
    # x_in = float(raw_input("enter required X"))
    # y_in = float(raw_input("enter required Y"))
    # z_in = loat(raw_input("enter required Z"))

    sphere_pose.pose.position.x = x_in
    sphere_pose.pose.position.y = y_in
    sphere_pose.pose.position.z = z_in

    scene.add_sphere(object_name, sphere_pose, radius=0.025)

if __name__ == '__main__':
    # init and home
    psm,group,scene = mm.init_and_home()

    # allign both dvrk and MC
    mm.update_mc_start_go(psm,group)

    # read psm 3D coordinate
    _ = mm.read_display_dvrk_pos(psm,verbose=True)

    # add object(s)
    raw_input("Adding the sphere...")
    add_sphere(scene,frame_id="psm_tool_tip_link", object_name="sphere1")

    # generate a trajectory which would normall hit the obstacle in the way
    cur = mm.read_display_dvrk_pos(psm)
    
    # set a goal position to move diagonally (10cm in negative Y and 10cm in negative X)
    traj = mm.generate_traj(p=psm,goal_coords_list=[[cur[0]-0.1, cur[1]-0.1, cur[2]]],total_points=5000,group=group,fixed_orientation='vertical')
    
    # start reading the goal+ actual positions in the background
    logger = start_recording(psm)

    # play the traj
    raw_input("Enter to move 10cm in +ve Y direction")
    mm.smooth_and_play(p=psm, traj=traj, smooth_factor=5)

    # stop the position recording an save the data collected
    stop_recording(logger, plot=False)

    raw_input("press enter to close")
    psm.shutdown()

