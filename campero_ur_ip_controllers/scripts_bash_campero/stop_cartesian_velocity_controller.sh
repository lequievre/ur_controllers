#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:[], stop_controllers:[cartesian_velocity_control], strictness: 2}"


