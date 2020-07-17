# ur_controllers
This repository contains controlers using ros_control architecture for an Universal Robot arm

# Position based cartesian velocity controller

This controller uses the *PositionJointInterface* of UR robots that is part of *Universal_Robots_ROS_Driver*: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
This controller uses the tools of *ROS Control*: http://wiki.ros.org/ros_control
This controller uses the tools of *KDL*: http://wiki.ros.org/kdl

This controller was originally forked from CentroEPiaggio/kuka-lwr (Manuel Bonilla, Carlos J. Rosales, Marco Esposito), license BSD.

## Description

The controller implements a cartesian position control. The input is a desired position (translation and orientation) value to be published on topic "command".
It uses KDL solvers (notably inverse kinematics solver) in order to turn this desired position into a target joint position sent to the interface.

## Topics

### command
The controller subscribes to this topic to get the desired value of cartesian position and orientation specified by the user
Type: PoseRPY

Position and Orientation :
rostopic pub -1 /campero/arm/cartesian_velocity_control/command  campero_ur_ip_controllers/PoseRPY '{id: 0, position: {x: 0.5, y: 0.5, z: 0.8}, orientation: {roll: 1.8, pitch: -0.3, yaw: 1.3}}'

Position only :
rostopic pub -1 /campero/arm/cartesian_velocity_control/command  campero_ur_ip_controllers/PoseRPY '{id: 1, position: {x: 0.27, y: 0.48, z: 0.88}}'

Orientation only :
rostopic pub -1 /campero/arm/cartesian_velocity_control/command  campero_ur_ip_controllers/PoseRPY '{id: 2, orientation: {roll: 0.2, pitch: 0.0, yaw: 0.0}}'

### current_x
The controller publishes on this topic to get the current pose of the end-effector
Type: geometry_msgs::Pose (with realtime_tools)

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

rostopic pub -1 /campero/arm/vel_based_cartesian_velocity_control/cmd_vel geometry_msgs/Twist -- '[0.0, 0.01, 0.0]' '[0.0, 0.0, 0.0]'

### x_master
The controller publishes on this topic to get the current pose of the end-effector
Type: geometry_msgs::Pose (with realtime_tools)

### x_dot_master
The controller publishes on this topic to get the current twist of the end-effector
Type: geometry_msgs::Twist (with realtime_tools)
