#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:['active_stiffness_control'], stop_controllers:[], strictness: 2}"


