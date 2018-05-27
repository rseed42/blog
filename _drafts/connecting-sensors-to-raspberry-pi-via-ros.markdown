---
layout: post
title: Connecting Sensors to Raspberry PI via ROS
---

# Accelerometer MPU6050

https://github.com/chrisspen/ros_mpu6050_node

### Install dependencies:

#### Transforms Libraries

```
rosinstall_generator tf --rosdistro kinetic --deps --wet-only --tar > kinetic-tf-wet.rosinstall
```

```
wstool merge -t src kinetic-tf-wet.rosinstall
```

```
wstool update -t src
```
```
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y  
```

Build everything (building individual packages in this case makes no sense, because there are quite a few new dependencies)
```
./src/catkin/bin/catkin_make_isolated -j1 --install --install-space /opt/ros/kinetic --pkg ${PACKAGE_NAME} -DCMAKE_BUILD_TYPE=Release
```



####


```
sudo git clone https://github.com/chrisspen/i2cdevlib.git
```


###

Unfortunately, the author of the library did not configure the package to be installed properly. In order to install it manually, copy all files under directory
```
install_isolated
```

to their proper location in 

```
/opt/ros/kinetic
```

### Start

To start the accelerometer node, you need to be root. Issue the following command:

```
roslaunch ros_mpu6050_node mpu6050.launch
```

