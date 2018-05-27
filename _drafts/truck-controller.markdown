---
layout: post
title: Truck Controller
---

# Initializing the ROS project
The truck controller is a new node that received commands from several control topics and generates the servo signals that drive the truck. The project should be located in the catkin workspace:

```
cd ros_catkin_ws/src
```

We need to initialze the project structure with the `catkin_create_pkg` command first. There is a dependency on the ROS C++ library:
```
catkin_create_pkg truck_controller roscpp
```
Let's modify `package.xml` so that we can build it properly.

# Prepare the Project Files
## package.xml
Set the project version
```
<version>0.0.1</version>
```

```
<description>Servo control node for driving the MAN model truck</description>
```

```
<maintainer email="petkov.venelin@gmail.com">rseed42</maintainer>
```

```
<license>Apache</license>
```
## controller_node.cpp
Add the source code template to `truck_controller/src/truck_controller_node.cpp`.
## CMakeLists.txt
The project build file has to be adjusted so that all components can be built properly.

Enable C++11 compatibility if it is used with more complex projects:
```
add_compile_options(-std=c++11)
```
```

```
catkin_package(
  DEPENDS roscpp
)
```
add_executable(distance_node src/distance_node.cpp)
```

```
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
)
```

Uncomment
```
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

```
install(FILES
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

## truck_controller.launch












