# dvrk-meng
The programmatic side of my masters research project. Here's what I'll cover:

- [The Big Project](#The-Big-Picture) (The idea of the wider research project)
- [My Part](#My-Part) (My role within the wider project)
- [This Repo](#This-Repo) (An overview of what this repository contains)
- [Setting Up the Environment](#Setting-Up-the-Environment)
- [Running the PSM Simulation](#Running-the-PSM-Simulation)
- [Setting up Moveit! for the PSM](#Setting-up-Moveit!)
- [Running-Moveit!](#Running-Moveit!)
- [Aliases](#Aliases) (shortcuts to run things faster)

<br/>

## The Big Picture

Perhaps you've heard of the da Vinci Surgical Robot? It's the platform that revolutionised surgery by allowing surgeons to remotely control tiny grippers inside the patient through minimally invasive incisions. This provides them with an unmatched level of inter-operative accuracy.

As a team of 4, we worked on developing a new feature for that surgical platform. The idea involved having one of its robotic arms carry an ultrasound probe and navigate autonomously to gather images of the region being operated on. We built a Convolutional Neural Network deep learning model that takes in these ultrasound images and is able to find blood vessels in them. With this information, we then developed a 3D reconstruction algorithm that takes these images of the detected vessel and creates a 3D model of it.

What's the point of all that? If a self-navigating arm is able to gather information about specific blood vessels close to the region being operated on, the surgeon becomes much more informed on the area and hence much less likely to damage any high-risk structures, such as the hepatic artery when performing a partial hepatectomy.


<br/>

## My Part

My specific role was being in charge of the control system. 

I created an algorithm that can generate efficient trajectories for the da Vinci manipulator, as well as an algorithm that smooths out these trajectories to avoid any jerky movements. The paths generation logic I developed could take into account obstacles, such as other da Vinci arms, and avoid them effectively. 

I also created a system that considers the location that the surgeon is currently operating at and uses that to automatically define both a region of interest and a search path inside of it. My research proposed the framework required for the arm to communicate with the CNN model for detecting the blood vessel from each ultrasound image, and discussed how this will help the autonomous arm make its decisions.

<br/>

## This Repo

This repository contains all the code I developed for the control system module I was in charge of, as well as all links to all the configuration files required. All the work was done in simulation using the dVRK Patient Side Manipulator (PSM).

This README file contains advice on setting up the environment and tools necessary. The actual code can be found in the `src/` directory, along with another readme that will guide you through some details of what's there.

<br/>

## Setting up the Environment
1. Install ubuntu on a virtual machine (I used virtualbox).
    > NOTE: The latest version of the cisst/SAW libraries needed are not compatible with the latest version of Ubuntu. I recommend installing Ubuntu 18.04 NOT 20.
2. Install ROS by following the tutorial [here](http://wiki.ros.org/Installation/Ubuntu). Make sure you select the right ROS version based on your Ubuntu version.
3. Follow the instructions [here](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros) to install and build the cisst/SAW libraries.
    > Note: Make sure `catkin build` ran successfully
4. Navigate into the `~/catkin_ws/src` directory and clone the dvrk-ros repo from [here](https://github.com/jhu-dvrk/dvrk-ros).
5. Run `catkin build` again, followed by `source devel/setup.bash`.

To make sure everything has installed successfully:
- `catkin build` should have given no errors
- Run `python` on the command line and `import dvrk`. If no errors are given, you're good to go!

<br/>

## Running the PSM Simulation

To visualise the PSM on RViz, follow the instruction in the dvrk_python directory [here](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_python).
The command to run the simulation requires providing a JSON configuration that describes the PSM. The JSON config file used in the work is in 
```
~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json'
```
Which makes the full command to run:
```
roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/ziad/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json
```
This should launch two GUIs - one with RViz showing the simulated PSM, and a QT window showing graphs that describe the motion of the PSM. If you see these then you're on the right track!

<!-- <image of GUIS> -->

<br/>

## Setting up Moveit!

Moveit! is an elaborate library that streamlines the path planning process for robot manipulators. The only thing is, you have to educate moveit with details about the dVRK PSM, as it needs to know details about the joints and links to be able to generate paths effectively.

This is done by creating a moveit configuration file using the [moveit setup assistant](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html). This process requires a URDF file specific to the PSM that includes details about the maipulator's joints and links. You have two options:

1. Use the setup assistant to generate a moveit configuration file. I included the URDF file to use in this repo, called `psm_modified.urdf`
2. You can use the moveit configuration I created for this project and just thank me later 😉. This is the folder called `psm_moveit_config` (the whole directory is the configuration).

<br/>

## Running Moveit!

Once the configuration file is ready, you can try running moveit to make sure things are fine. 

### Running the Moveit GUI

Make sure you run this from the same directory as the `psm_moveit_config` folder that is the configuration we just created.
```
roslaunch psm_moveit_config demo.launch
```
This should launch GUI that shows the simulated PSM. The moveit GUI is not used for this study, but we're trying it here to make sure everything is intact.
<br/>

### Running the Moveit API

To only expose the moveit API for using it prigrammatically (which is what my work is using), you can start moveit without the GUI:
```
roslaunch psm_moveit_config demo.launch use_rviz:=false
```
I avoided running the RViz GUI as it slowed down the ubuntu virtual machine without bringing much benifit. The dvrk-ros library already simulates the PSM and shows it in RViz.

<br/>

## Aliases

If you are lazy as me, you'll enjoy these aliases I used to streamline calling the commands to spin up the dvrk and moveit simulations. Aliases are basically command line shortcuts which you can add to your `~/.bashrc` file.
```
# Run the DVRK simulation with the default JSON
alias runsim='roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/ziad/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json'

# kill both roscore and dvrk rviz
alias killall='pkill roslaunch;pkill roscore'

# run the moveit API
alias move_api='roslaunch psm_moveit_config move_group.launch &'

# run the moveit GUI
alias move_gui='roslaunch psm_moveit_config demo.launch'

# run moveit demo without rviz (optimal)
alias move='roslaunch psm_moveit_config demo.launch use_rviz:=false'

# run roscore and launch rviz for dvrk in background
alias runall='roscore & (sleep 2; runsim)& (sleep 4; move)&'

# run roscore and launch rviz for dvrk in background AND use moveit's GUI
alias runallgui='roscore & (sleep 2; runsim)& (sleep 4; move_gui)&'

# reset the running nodes
alias rstall='killall & (sleep 3;runall)&'

# reset the running nodes with moveit's GUI
alias rstallgui='killall & (sleep 3;runallgui)&'
```
