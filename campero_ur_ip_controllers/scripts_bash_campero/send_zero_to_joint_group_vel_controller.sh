#!/bin/bash


rostopic pub -1 /campero/arm/joint_group_vel_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0,0.0,0.0]"


