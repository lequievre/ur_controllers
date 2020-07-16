#!/bin/bash

rosservice call /campero/arm/controller_manager/switch_controller "{start_controllers:[], stop_controllers:[active_stiffness_control], strictness: 2}"


