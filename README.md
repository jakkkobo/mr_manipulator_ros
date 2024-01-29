# mr_manipulator_ros

## Description
This ROS1 (Noetic) node provide a implementation of a reactive robot wall follower

## Dependencies

- [ros_noetic](http://wiki.ros.org/noetic/)
- [Gazebo](http://wiki.ros.org/gazebo)
- [Rviz](http://wiki.ros.org/rviz)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [std_msgs](http://wiki.ros.org/std_msgs)
- [interactive_markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
- [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)


## Usage

### Compilation

Create workspace if you don´t have one:
````
mkdir catikin_ws
cd catkin_ws
mkdir src
````

Clone [mr_manipulator_ros](https://github.com/jakkkobo/mr_manipulator_ros.git) in src folder and compile:
````
cd src
git clone https://github.com/$USER/mr_manipulator.git
cd catkin_ws
catkin_make
````

### Files structure
````


````

### Launch

````
roslaunch turtlebot3_control bringup_robot.launch
````

### Dynamic Reconfigure Parameters

To adjust the robot parameters and send commands in real-time run:

````
rosrun rqt_reconfigure rqt_reconfigure

````

### Graphs visualization
To visualize the robot pose, heading, velocities and accelerations during the navigation run. (launchs by default)
