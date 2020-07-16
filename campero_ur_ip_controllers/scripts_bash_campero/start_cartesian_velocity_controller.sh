#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:['cartesian_velocity_control'], stop_controllers:[], strictness: 2}"


