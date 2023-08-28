# ROS Tutorial
[reference](https://www.youtube.com/watch?v=Qk4vLFhvfbI&list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q&ab_channel=RoboticsBack-End)

## 1. Install and Setup ROS Noetic 
[wiki.ros](http://wiki.ros.org/noetic/Installation/Ubuntu)

**Environment setup**
```
source /opt/ros/noetic/setup.bash
```

**Dependencies for building packages**
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

**Run the ROS**
```
roscore
```


## 2. Understand What is a ROS Node

+ window 1: `rosrun rospy_tutorials talker`
+ window 2: `rqt_graph`
+ window 3: `rosrun rospy_tutorials listener`
+ simulator: `rosrun turtlesim turtlesim_node`
+ simulator: `rosrun turtlesim turtle_teleop_key`



## 3. Create and Setup a Catkin Workspace
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
source ~/ros_projects/catkin_ws/devel/setup.bash
```

## 4. Create a ROS Package
```
cd catkin_ws/src/
catkin_create_pkg my_robot_controller rospy turtlesim
rospack list | grep my_robot_controller
```

visual studio:
```
sudo snap install code --classic
code .
```



## 5. Write a ROS Node
```
cd ~/catkin_ws/src/my_robot_controller
mkdir scripts
tourch my_first_node.py
chmod +x my_first_node.py
rosrun my_robot_controller my_first_node.py
```

+ Check the node: `rosnode list`
+ Kill the node: `rosnode kill /test_node`


## 6. ROS Topic
+ window 1: `rosrun rospy_tutorials talker`
+ window 2: `rosrun rospy_tutorials listener`
+ window 3: 
```
rostopic list
rostopic info /chatter
rosmsg show std_msgs/String
rostopic echo /chatter
```

## 7. ROS Publisher
```
rosrun turtlesim turtlesim_node
rostopic list
rostopic info /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist
```
then fix the package.xml
```
rosrun my_robot_controller draw_circle.py
```

check the topic:
```
rostopic info /turtle1/cmd_vel
```

## 8. ROS Subscriber
```
rosrun turtlesim turtlesim_node
rostopic echo /turtle1/pose
touch pose_subcriber.py
```

## 9. Combine Publisher and Subscriber in a Closed Loop System
```
touch turtle_controller.py
```

## 10. ROS Service
run service
```
rosrun rospy_tutorials add_two_ints_server
```
check services:
```
rosservice list
rosservice info /add_two_ints
rosservice call /add_two_ints 2 8
```

From `rosservice info /add_two_ints`, we have Type: `rospy_tutorials/AddTwoInts`. To check more info:
```
rossrv show rospy_tutorials/AddTwoInts
```

Simulator:
```
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rosservice call /turtle1/set_pen <TAB>
```

## 10. ROS Service Client
