#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:['joint_group_vel_controller'], stop_controllers:[], strictness: 2}"


