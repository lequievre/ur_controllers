#!/bin/bash

rosservice call /campero/arm/ur_hardware_interface/dashboard/load_program externalControl.urp
sleep 1.0
rosservice call /campero/arm/ur_hardware_interface/dashboard/play


