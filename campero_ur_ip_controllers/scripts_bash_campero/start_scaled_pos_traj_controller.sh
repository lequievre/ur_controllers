#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:['scaled_pos_traj_controller'], stop_controllers:[], strictness: 2}"


