#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:[], stop_controllers:['scaled_pos_traj_controller'], strictness: 2}"


