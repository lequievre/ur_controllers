

1- Add controller properties in a yaml file
===========================================

/home/campero/catkin_ws/src/campero/campero_common/campero_control/config
/ur10_controllers.yaml


for exemple :
-------------

# Laurent LEQUIEVRE
cartesian_velocity_control:
   type: campero_ur_ip_controllers/CartesianVelocityControl
   root_name: campero_ur10_base_link
   tip_name: campero_ur10_wrist_3_link




2- Add the controller name to "stopped controllers" in the launch file
======================================================================

/home/campero/catkin_ws/src/campero/campero_robot/campero_bringup/launch/ur10_complete.launch

for exemple :
-------------

<arg name="stopped_controllers" default="pos_traj_controller cartesian_velocity_control active_stiffness_control" doc="Controllers that are initally loaded, but not started."/>
