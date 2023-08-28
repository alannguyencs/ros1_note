# Jackal
[source](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)

## Installation on ROS-Noetic
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

## Frequent Commands
```
conda deactivate
cd ~/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/
source ~/GoogleDrive/CAiRS//ros_projects/catkin_ws/devel/setup.bash
```

start robot by keyboard: `roslaunch jackal_tools start_teleop.launch`

stop robot running:
`rostopic pub -r1 /cmd_vel geometry_msgs/Twist <double TAB>`

# Jackal
[source](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)

## Installation on ROS-Noetic
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

## Frequent Commands
```
conda deactivate
cd ~/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/
source ~/GoogleDrive/CAiRS//ros_projects/catkin_ws/devel/setup.bash
```

start robot by keyboard: `roslaunch jackal_tools start_teleop.launch`

stop robot running:
`rostopic pub -r1 /cmd_vel geometry_msgs/Twist <double TAB>`



# LAUNCH GAZEBO
Default: `roslaunch jackal_gazebo jackal_world.launch`

In empty world: `roslaunch jackal_gazebo empty_world.launch`

With LiDAR:
```
export JACKAL_LASER=1
roslaunch jackal_gazebo jackal_world.launch
```

With LiDAR and StereoCamera:
```
export JACKAL_LASER=1
export JACKAL_BB2=1
roslaunch jackal_gazebo jackal_world.launch
```

With LiDAR and RGB Camera:
```
export JACKAL_LASER=1
export JACKAL_FLEA3=1
roslaunch jackal_gazebo jackal_world.launch
```
Note that for now we are unable to set StereoCamera and RGB Camera at the same time.

Other settings can be found at [description package](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/description.html)



## Visualize on RVIZ
```
roslaunch jackal_viz view_robot.launch
```

## Additional Simulation Worlds and Sensors setting
[source](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/additional_sim_worlds.html)

## Navigation and human avoidance courses
The construcsim [Jackal robot](https://app.theconstructsim.com/courses/mastering-with-ros-jackal-19/)

## Unit 0: Introducing ClearPath Jackal Robot
Demo Move Around
```
rostopic pub -r1 /cmd_vel geometry_msgs/Twist <double TAB>
```


## Unit 1: Navigation Indoor
### How to Move the Robot
```
cd ~/ros_projects/catkin_ws/src/
catkin_create_pkg my_jackal_tools rospy
source ~/ros_projects/catkin_ws/devel/setup.bash
touch my_jackal_tools/scripts/draw_circle.py
```

To move the robot in circle:
```
rosrun my_jackal_tools draw_circle.py
```

To let the robot stop:
```
rostopic pub /cmd_vel geometry_msgs/Twist [TAB][TAB]
```

## PointCloud
To establish point cloud calculated from stereo camera: `roslaunch my_robot_controller start_stereovision.launch`

## Mapping
To scan and save a map:
+ `roslaunch jackal_gazebo jackal_world.launch`
+ `roslaunch my_jackal_tools start_mapping.launch`
+ `rosrun rviz rviz`
+ load rviz config, set `Fixed Frame = Map`

To save the map:
```
roscd my_jackal_tools
mkdir maps
cd maps
rosrun map_server map_saver -f mymap
```

To run robot with map:
+ `roslaunch jackal_gazebo jackal_world.launch`
+ `rosrun rviz rviz`
+ `roslaunch my_jackal_tools start_navigation_with_map.launch`
+ load rviz config, set `Fixed Frame = Map`, add topic `/particlecloud`
+ Localise robot location:
    + use button `2D Pose Estimation` to indicate where it is in the global map [url](https://www.youtube.com/watch?v=7qiew0powLc&ab_channel=rst)
    + use button `2D Nav Goal` to let the robot move in short distance



## To load and run jackal in custom world
+ create `world` file and `launch` file
+ `export JACKAL_LASER=1`
+ `export JACKAL_BB2=1`
+ `cd ~/GoogleDrive/CAiRS/ros_projects/catkin_ws/`
+ `catkin_make`
+ `source devel/setup.bash`
+ `roslaunch studying_gazebo demo_sdf_world.launch`
+ `roslaunch my_jackal_tools start_mapping.launch`

