#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:[], stop_controllers:['joint_group_vel_controller'], strictness: 2}"


