# Turtlebot 3
## Activate environment
```bash
conda deactivate
cd ~/GoogleDrive/CAiRS/ros_projects/catkin_ws/
catkin_make
source devel/setup.bash
```

## Navigation
### Launch the simulation
```bash
roslaunch studying_gazebo turtlebot3_world_I.launch
```
### Launch the navigation within a map
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/my_turtlebot3_tools/maps/world_I_1.yaml
```

### Stop moving
```bash
rostopic pub /cmd_vel geometry_msgs/Twist [TAB][TAB]
```

### Moving through some pre-defined points:
```bash
rosrun my_robot_controller 230822_odom_feedback_control.py
```


