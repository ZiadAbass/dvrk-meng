# moveit! configuration for the dVRK PSM

This configuration is based on the `psm_modified.urdf` from the [dvrk_motion_planning](https://github.com/Stormlabuk/dvrk_motion_planning) repo. It was taken and placed into the cloned [dvrk_env](https://github.com/WPI-AIM/dvrk_env) repo.


## Launching moveit GUI: 

> To launch the moveit GUI with the simulation. This is necessary if you want to visualise the arm planning and moving. Also needed to find the current position of the arm. Still not sure if the read position through moveit is always the same as the read position using the dvrk simulation. 

`roslaunch psm_moveit_config demo.launch` 

 
## Launching the moveit commander API 

> This is necessary to be able to communicate with the moveit_commander API. This loads the PSM URDF onto the paremeter server. This makes ROS aware of the details of the robotic arm being controlled. You do not have to run this if you run the demo.launch as above. 

`roslaunch psm_moveit_config move_group.launch` 

 
Once either of these is run, python can be used to communicate with moveit using the moveit_commander API (http://docs.ros.org/en/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html). 
