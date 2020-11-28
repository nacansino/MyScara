# SCARA Robot Simulation in Gazebo with ROS (WIP)

## Installing ROS
- http://wiki.ros.org/ROS/Installation
- We used ros-noetic

## How I built the code

### 1 .Building the ROS directory using `catkin_make`
```bash
cd ~
mkdir -p myscara/src # -p option also creates the parent directories
cd myscara
catkin_make
echo "source ~/.myscara/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
```

### 2. Create the gazebo world as a ROS package (catkin_create_pkg)
```bash
cd ~/myscara/src
catkin_create_pkg myscara_gazebo
```
Further, we need the description of the gazebo world. Create a file called `myscara.world` in `src/myscara_gazebo/worlds/` 

```
<?xml version="1.0" ?>
<sdf version="1.4">
  <!-- We use a custom world for the rrbot so that the camera angle is launched correctly -->

  <world name="default">
    <include>
      <uri>model://willowgarage</uri> <!-- garage -->
    </include>

    <include>
      <uri>model://ground_plane</uri> <!-- floor -->
    </include>

Copyright (C) 2019 Artifex Software, Inc.  All rights reserved.
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Focus camera on tall pendulum -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>

```

### 3. Create the robot as a ROS package
```bash
cd ~/myscara/src
catkin_create_pkg myscara_description   # we call this description because it describes our robot
```
We need the [urdf](http://wiki.ros.org/urdf) that represents our robot model. This is different from the [sdf](https://newscrewdriver.com/2018/07/31/ros-notes-urdf-vs-gazebo-sdf/#:~:text=URDF%20is%20the%20established%20format,information%20within%20tags.) which is a format used by Gazebo to describe the simulation world (can also be used to describe robot models).

### We bring in 3 files (located inside `src/myscara_descirption/urdf`): 
- myscara.xacro     # Main file. Includes the other three files below.
- myscara.gazebo    # Describes the connection between ROS and Gazebo. 
- macros.xacro
- materials.xacro

### 4. Launch gazebo through `roslaunch`
```bash
roslaunch myscara_gazebo myscara_world.launch
```

Note that we also made a special file that includes some boilerplate commands for cleaining up:
```bash
sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
roslaunch myscara_gazebo myscara_world.launch
```

### 5. Try to control the robot manually by publishing to the ros `/cmd_vel` topic

Try to publish commands to control the robot:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.1
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0"
```

### 6. Connecting RViz

To connect to RViz, create the file `myscara_rviz.launch`.

```xml
<?xml version="1.0"?>
<launch>

  <param name="robot_description" command="$(find xacro)/xacro '$(find myscara_description)/urdf/myscara.xacro'"/>

  <!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="False"/>
  </node>

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz"/>
  <!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find mybot_description)/launch/myrobot.rviz"/-->

</launch>


```
Try to launch RViz:

```bash 
#!/bin/bash                                                                     

roslaunch myscara_description myscara_rviz.launch

```

## Useful ROS commands
1. rqt_graph - shows the node graph of ROS messages


## Pitfalls

1. RLException: [xxx.launch] is neither a launch file in package...
- This is most likely caused when you created a new launch file but has not rerun the `deve/setup.bash` script inside the workspace.

## ToDo
- Create SDF for the SCARA robot arm using the models in `scaramodels/`
- Add custom end-effector for the spindle disposer. 
- Create sample end-effector controller.
- Connect the end-effector controller to a G-Code parser (can use parts of [GRBL](https://github.com/grbl/grbl)).

## Potential References

1. https://github.com/yangliu28/two_scara_collaboration#progress-and-problems-may-3-2016
2. http://gazebosim.org/tutorials?tut=build_model
3. https://howtomechatronics.com/projects/scara-robot-how-to-build-your-own-arduino-based-robot/