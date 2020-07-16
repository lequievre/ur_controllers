# ur_controllers
This repository contains controlers using ros_control architecture for an Universal Robot arm


# Velocity based cartesian velocity controller

This controller uses the *VelocityJointInterface* of UR robots that is part of *Universal_Robots_ROS_Driver*: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
This controller uses the tools of *ROS Control*: http://wiki.ros.org/ros_control
This controller uses the tools of *KDL*: http://wiki.ros.org/kdl

## Description

The controller implements a cartesian velocity control. The input is a desired twist value to be published on topic "cmd_vel".
It uses KDL solvers (notably inverse kinematics solver) in order to turn this desired twist into a target joint velocity sent to the interface.

## Topics

### cmd_vel
The controller subscribes to this topic to get the desired value of cartesian velocity specified by the user
Type: geometry_msgs::Twist

### x_master
The controller publishes on this topic to get the current pose of the end-effector
Type: geometry_msgs::Pose (with realtime_tools)

### x_dot_master
The controller publishes on this topic to get the current twist of the end-effector
Type: geometry_msgs::Twist (with realtime_tools)
