'''
Abbreviations:
OPSM -> Operating PSM, the PSM that is controlled by the surgeon which is performing the resection.
UPSM -> Ultrasound PSM, the PSM carrying the US probe being controlled by this system.

Point in Time:
- Just finished bounding_rectangle script which takes in an OPSM 
end effector pose and retrieves a search path for the UPSM to follow.
- Want to now test this using simulated OPSM end effector pose and the actual UPSM

Purpose:
Provide a fake 2D location and Z axis rotation and use bounding_rectangle to generate a rectangle as well
as a search path. This search path is then fed to the UPSM and it should explore that box by following
that path in a single trajectory.
'''

import master as mm
import bounding_rectangle as br
import PyKDL
import time
import cubic_smoothing as cs
import numpy as np

'''
- init and home
- read psm 3D coordinate
- simulate OPSM pose 
- provide that to bounding_rectangle and retrieve a path
- go to the first point on that path
- generate a trajectory from the retrieved path
- follow the path
'''

# define number of columns we want to search through
search_cols = 5

if __name__ == '__main__':
    # - init and home
    psm,group = mm.init_and_home()

    # - read psm 3D coordinate
    xyz = mm.read_display_dvrk_pos(psm,verbose=False)

    # - simulate OPSM pose, assume it is a few cm away and rotated 40 degrees
    OPSM_sim_xy = [xyz[0]+0.03, xyz[1]+0.02]
    OPSM_sim_rot = 40

    # - provide that to bounding_rectangle and retrieve a path
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
    
    # - follow the path with multipl goto's [OLD method]
    # raw_input("Press enter to follow the points")
    # for idx,step in enumerate(path_3D):
    #     mm.goto_xyz(psm, goal_coords=goal, total_points=5000, group=group,fixed_orientation='vertical')
    #     time.sleep(0.5)

    # - generate a trajectory from the retrieved path and follow it [NEW METHOD]
    raw_input("Press enter to follow the traj")
    smoothed_traj = mm.goto_multiple_xyz(psm, goal_coords_list=path_3D, total_points=20000, group=group,fixed_orientation='vertical')

    # close
    time.sleep(1)
    psm.shutdown()