
# UAV-UGV Vision System

**Author:** Aurélien Garreau  
**Co-author:** *(to be completed)*

---

## Dependencies

Install the required packages:

    sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
    pip install opencv-contrib-python
    *(to be completed)*

---

## Launching the Program

### In Simulation

1. Open the file `vision_launcher.launch` in the `launch_uav_ugv` package.
2. Ensure the following line is set:

       <arg name="in_simu" default="true" doc="Set to true for simulation (uses localhost)" />

3. Then, run:

       flairrun uavcoop
       roslaunch launch_uav_ugv vision_launcher.launch

---

### In Flight Area (Real Hardware)

1. Open the same file: `vision_launcher.launch` in the `launch_uav_ugv` package.
2. Set the following values (make sure the IP address is correct):

       <arg name="in_simu" default="false" doc="Set to true for simulation (uses localhost)" />
       <arg name="host" default="172.26.209.28" doc="TCP server IP address (ignored if in_simu is true)" />

---

## ROS Environment Configuration

### If Using the UGV (Remote Setup)

Edit your `~/.bashrc`:

       nano ~/.bashrc

Update or add the following lines:

       # Simulation setup (commented out)
       # export ROS_IP=localhost
       # export ROS_HOSTNAME=localhost
       # export ROS_MASTER_URI=http://localhost:11311

       # UGV setup
       export ROS_IP=172.26.213.70
       export ROS_HOSTNAME=172.26.213.70
       export ROS_MASTER_URI=http://172.26.217.33:11311

Reload and verify:

       source ~/.bashrc
       printenv | grep ROS

---

### Else (Localhost / Simulation)

Edit your `~/.bashrc`:

       nano ~/.bashrc

Ensure it contains:

       export ROS_IP=localhost
       export ROS_HOSTNAME=localhost
       export ROS_MASTER_URI=http://localhost:11311

       # UGV configuration (commented out)
       # export ROS_IP=172.26.213.70
       # export ROS_HOSTNAME=172.26.213.70
       # export ROS_MASTER_URI=http://172.26.217.33:11311

Reload and verify:

       source ~/.bashrc
       printenv | grep ROS

---

## Building and Sourcing the Workspace

### Build After Modifying the Code

       cd ~/ros1_ws
       catkin_make
       source devel/setup.bash

### Add to `.bashrc` for Convenience

To avoid sourcing manually every time, add these lines to your `.bashrc`:

       nano ~/.bashrc

At the end of the file, add:

       source /opt/ros/noetic/setup.bash
       source /home/aurelien/flair/my_src/ros1_ws/devel/setup.bash

---

## Command Summary

       flairrun uavcoop
       roslaunch launch_uav_ugv vision_launcher.launch