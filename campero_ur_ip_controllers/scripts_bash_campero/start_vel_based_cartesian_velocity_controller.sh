#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:['vel_based_cartesian_velocity_control'], stop_controllers:[], strictness: 2}"
