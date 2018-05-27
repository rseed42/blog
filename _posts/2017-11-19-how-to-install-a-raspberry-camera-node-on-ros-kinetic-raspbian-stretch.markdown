---
layout: post
title: How to install a Raspberry Camera Node on ROS Kinetic (Raspbian Stretch)
date: '2017-11-19 13:16:54'
---

# Raspicam Installation Tutorial

In order to use the Raspberry Pi 3 camera v2, we need to install a third-party ROS node from source, since it is not part of the ROS distribution at the moment. The installation is not that straightforward using only the barebones ROS installation, since there are a few dependencies on other packages. Looking at the package definition [package.xml](https://github.com/UbiquityRobotics/raspicam_node/blob/indigo/package.xml), we see the following dependencies:

* catkin
* `compressed_image_transport`
* roscpp
* std_msgs
* std_srvs
* sensor_msgs
* `camera_info_manager`
* `dynamic_reconfigure`
* libraspberrypi0

The highlighted ones are missing from the `ros_comm` stack, so we need to install them manually. The approach here is simply to fetch the missing packages and then merge them into the existing barebones catkin workspace. Lastly, we build and test `raspicam_node`.

###### 1. Install all dependencies

Fetch the package information for all the missing packages and their ROS dependencies:

```
rosinstall_generator compressed_image_transport --rosdistro kinetic --deps --wet-only --tar > kinetic-compressed_image_transport-wet.rosinstall

rosinstall_generator camera_info_manager --rosdistro kinetic --deps --wet-only --tar > kinetic-camera_info_manager-wet.rosinstall

rosinstall_generator dynamic_reconfigure --rosdistro kinetic --deps --wet-only --tar > kinetic-dynamic_reconfigure-wet.rosinstall
```
Now we need to fetch the sources and put them to the `~/ros_catkin_ws/src` where all the other packages from the barebone installation are located:
```
wstool merge -t src kinetic-compressed_image_transport-wet.rosinstall
wstool merge -t src kinetic-camera_info_manager-wet.rosinstall
wstool merge -t src kinetic-dynamic_reconfigure-wet.rosinstall
```
```
wstool update -t src
```
Fetch any additional Raspbian libraries that are needed
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y  
```
Build the packages. Please, note that this takes a very long time, so it might be a good idea to build it overnight in a tmux window.
```
./src/catkin/bin/catkin_make_isolated -j1 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release
```
It turns out that `raspicam_node` depends on the raspberry pi library, so we also install the headers:
```
sudo apt-get install libraspberrypi-dev
```
###### 2. Build the raspicam node
Check out the source code for `raspicam_node` from Github in the workspace src directory:
```
cd ~/ros_catkin_ws
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```
Install other library dependencies automatically:
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y  
```
Finally, build and install `raspicam_node`. It should be possible to do this more specifically with `--pkg raspicam` and save some time, but this hasn't been tried yet. Two compilation processes `-j2` are a safe option here:
```
./src/catkin/bin/catkin_make_isolated -j2 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release
```

###### 3. Test the camera
Now that we have the camera node installed, we can test the Raspberry camera if we haven't done that yet. It needs to be enabled with `raspi-config` from the `interface` menu:
```
sudo raspi-config
```
Take a test shot
```
raspistill -o test.jpg
```
Everything is fine, so we can test the raspicam node.

###### 4. Test raspicam_node
Start a new tmux session and source the setup file in every relevant window
```
source /opt/ros/kinetic/setup.bash
```
Open a new window for `roscore` and start it there. Find the launch definitions in 
`~/ros_catkin_ws/src/raspicam_node/launch/` and go there:
```
cd ~/ros_catkin_ws/src/raspicam_node/launch/
```
Start `raspicam_node` with the launch configuration of choice:
```
roslaunch camerav2_1280x960.launch
```
A simple topic check shows us that the node is active:
```
rseed42@raspi:~$ rostopic list
/raspicam_node/camera_info
/raspicam_node/image/compressed
/raspicam_node/parameter_descriptions
/raspicam_node/parameter_updates
/rosout
/rosout_agg
```
