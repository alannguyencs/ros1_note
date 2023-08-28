# Projects

## Custom world
[reference](https://www.youtube.com/watch?v=9zvg7GUOZk8&ab_channel=%ED%85%90%EC%B4%88)
+ `gazebo`
+ put some objects in the worlds
+ save world as `simple_world.world`
+ Ctrl + C to turn off the world
+ To open the world: `gazebo simple_world.world`

## Build a world
+ `roslaunch gazebo_ros empty_world.launch`
+ `rosrun studying_gazebo born_I.py`

## Add Turtlebot to the custom world
+ copy `turtlebot3_world.launch` to `simple_world_with_turtlebot.launch` and edit as:
```
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="-2.0"/>
  <arg name="y_pos" default="-0.5"/>
  <arg name="z_pos" default="0.0"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find studying_gazebo)/worlds/simple_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf"  args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
</launch>
```

+ `roslaunch studying_gazebo simple_world_with_turtlebot.launch`
+ To visualize the robot and laser scan in Rviz, add the following code to `simple_world_with_turtlebot.launch`:
```
<!-- TurtleBot3 -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)" />
  </include>
```
This part is found at `turtlebot3_slam/launch/turtlebot3_slam.launch`

## Turtlebot3 mapping and navigation in a custom world
Use the mapping and navigation documentation. It seems Turtlebot3 and Jackal have different launch files for their navigation.

Mapping
+ `roslaunch studying_gazebo turtlebot3_house.launch`
+ `rosrun rviz rviz`
+ `export TURTLEBOT3_MODEL=waffle`
+ `roslaunch my_turtlebot3_tools start_mapping.launch`
+ move with keyboard: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
+ `cd ~/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/my_turtlebot3_tools/maps/`
+ `rosrun map_server map_saver -f empty_turtlebot3_house`

Navigation
+ `roslaunch studying_gazebo turtlebot3_house.launch`
+ `rosrun rviz rviz`
+ `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/my_turtlebot3_tools/maps/empty_turtlebot3_house.yaml`
+ stop the robot: `rostopic pub /cmd_vel geometry_msgs/Twist [TAB][TAB]`
+ remark: on gazebo GUI, click on translation mark and move the robot to see the direction marker.
+ **Task: getting familiar with everything in Turtlebot3, don't use the code from Jackal anymore.**
+ **Problem: local_cost_map does not work, the robot hit obstacle**

## Service /gazebo/spawn_sdf_model
+ `roslaunch gazebo_ros empty_world.launch`
