# How I made this dir

## 1 .Building the ROS directory using `catkin_make`
```bash
cd ~
mkdir -p myscara/src # -p option also creates the parent directories
cd myscara
catkin_make
echo "source ~/.myscara/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
```

## 2. Create the gazebo world as a ROS package (catkin_create_pkg)
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

    <!-- Global light source -->
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

## 3. Create the robot as a ROS package
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

## 4. Launch gazebo through `roslaunch`
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

## 5. Try to control the robot manually by publishing to the ros `/cmd_vel` topic

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