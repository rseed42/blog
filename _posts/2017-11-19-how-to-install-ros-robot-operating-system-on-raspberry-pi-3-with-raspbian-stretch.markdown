---
layout: post
title: How to Install ROS (Robot Operating System) On Raspberry Pi 3 with Raspbian
  Stretch
date: '2017-11-19 00:17:12'
---

# Installation Instructions
An easier way to install ROS on Raspberry Pi is to use the Linux Mate distribution for Raspberry. However, I am unwilling to use bloated software, so here we do it the hard way :).

This guide describes how to install a barebone kinetic ROS distribution. The installation of additional packages is described in follow-up posts.

#### Preparation
We need to install some catkin bootstrap packages first. In order to do this, we must add the ros software repository. It seems that Raspbian Stretch is missing the certificate management service by default, so we have to install it first:

```
sudo apt-get install dirmngr
```
Now we can add the ROS repository to apt:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Add the public key:
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
Update apt:
```
sudo apt-get update
```
Install the bootstrap dependencies:
```
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
```
This will pull some other package dependencies, for instance `build-essentials`.

#### Installation
###### 1. Initialize the ROS package system
The first step is to initialize the *rosdep* package manager. As per the documentation, **do not use sudo** for `rosdep update`:
```
sudo rosdep init
rosdep update
```
###### 2. Create the catkin workspace
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```
###### 3. Build a list of packages and dependencies
The command `rosinstall_generator` is used to plan out the build process by generating a `*.rosinstall` file. It resolves all dependencies for the specified package and writes them down in the `rosinstall` file. That file is used by the `wstool` to fetch the source code for all packages.

The very minimal installation is based on the `ros_comm` package:
```
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
```
Here `--wet-only` describes that only catkin (the new build system) packages are to be fetched. If you don't want to start with the very basic system, there are a few other options available:

* robot
* perception
* move-arm
* simulators
* viz

For more info check the documentation at the [ROS Wiki](http://ros.org/reps/rep-0131.html#variants).

###### 4. Fetch the packages

The command `wstool` is used to obtain the source code automatically. It can download the packages in parallel, controlled by the `-j` option. In this step, a higher parallelization (-j8) is fine:
```
wstool init -j8 src kinetic-ros_comm-wet.rosinstall
```
If the `wstool init` command fails or it gets interrupted, the download can be resumed as follows:
```
wstool update -j4 -t src
```
When finished, the source code for the required packages can be found in 
```
~/ros_catkin_ws/src
```
Even though we have downloaded the packages in `ros_comm`, there might be external dependencies (libraries and tools) that are not yet installed on the Raspbian OS. In order to fetch them automatically, we invoke the following command:
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```
It installs everything via the apt package manager. The dependecies are resolved for all ROS source packages in the `--from-paths` directory. Since the above command is recursive, we tell it via `--ignore-src` not to install the packages already in the src directory. As we are not working in the ROS environment yet, we have to also specify the distribution name with the `--rosdistro` parameter.

###### 5. Build the ROS packages
The command for building the packages has been fetched with wstool along with the source code and is located in the `src` directory. The `catkin_make_isolated` is used for a mix of catkin and cmake packages. If only catkin packages are to be built, the `catkin_make` should be used instead (custom-built ROS packages).

The Raspberry 3 has limited memory. With the default parallel setting `-j4`, it overflows and the build process fails. Empirically, `-j2` is a good parameter.

Finally, the install location can be specified with the `--make-space` parameter. In order to simplify the process, we can make the current user owner of the target directory where we want to install ROS:
```
sudo mkdir -p /opt/ros/kinetic
sudo chown rseed42:rseed42 /opt/ros/kinetic
```

Start the build command:
```
./src/catkin/bin/catkin_make_isolated -j2 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release
```
If there is a problem with the compilation (crash due to memory exhaustion), the command can be issued again and the packages that are already compiled will be skipped, so it continues from where it left off.

###### 6. Test the installation
ROS uses a few environment variables that have to be set up prior to using it. According to the build command above, we can find a setup script at:
```
source /opt/ros/kinetic/setup.bash
```
Check that the ROS environment variables are set:
```
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_MASTER_URI=http://localhost:11311
ROS_PACKAGE_PATH=/opt/ros/kinetic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=kinetic
```
We need to start the ROS core process as follows:
```
roscore
```
We can now check the default topics with:
```
rseed42@raspi:~/ros_catkin_ws$ rostopic list
/rosout
/rosout_agg
```
Everything seems fine, this was the easy part!




