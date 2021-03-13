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
that path.
'''

import master as mm
import bounding_rectangle as br
import PyKDL

'''
- init and home
- read psm 3D coordinate
- simulate OPSM pose 
- provide that to bounding_rectangle and retrieve a path
- go to the first point on that path
- generate a trajectory from the retrieved path
- follow the path
'''

if __name__ == '__main__':
    # - init and home
    psm,group = mm.init_and_home()

    # - read psm 3D coordinate
    xyz = mm.read_display_dvrk_pos(psm,verbose=False)

    # - simulate OPSM pose, assume it is a few cm away and rotated 40 degrees
    OPSM_sim_xy = [xyz[0]+0.03, xyz[1]+0.02]
    OPSM_sim_rot = 40

    # - provide that to bounding_rectangle and retrieve a path
    _, path = br.main(rec_width_in=0.10, rec_height_in=0.05, gripper_location=OPSM_sim_xy, gripper_rotation_degrees=OPSM_sim_rot, gripper_displacement=0.3, search_columns=4, height_padding=0.2, plot=True)
    
    # - go to the first point on that path
    path_beginning = [path[0][0],path[0][1],xyz[2]]
    raw_input("press enter to go to the path's beginning")
    mm.goto_xyz(psm, goal_coords=path_beginning, total_points=3000,group=group,fixed_orientation='vertical')

    # - generate a trajectory from the retrieved path

    # - follow the path
    for idx,step in enumerate(path):
        buf = "Press Enter to move to coordinate number %d" % (idx)
        raw_input(buf)
        # convert 2D into 3D using a fixed Z (height)
        goal = [step[0],step[1],xyz[2]]
        mm.goto_xyz(psm, goal_coords=goal, total_points=2000, group=group,fixed_orientation='vertical')

    # close
    psm.shutdown()