/*
 *  A ROS package to control the UR10 of the Campero robotic platform (both simulation and physical robot).
 *  With an array of joint values (message std_msgs/Float64MultiArray).
 * 
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
 * joint_array_control:
 *  type: campero_ur_ip_controllers/JointArrayControl
 *  publish_rate: 50
 *  root_name: campero_ur10_base_link
 *  tip_name: campero_ur10_wrist_3_link
 * 
 * rostopic pub -1 /campero/arm/joint_array_control/command  std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0,0.0,0.0]"
 *
 * 
 */

#ifndef UR_CONTROLLERS_JOINT_ARRAY_H
#define UR_CONTROLLERS_JOINT_ARRAY_H

// Controller base
#include "kinematic_chain_controller_base.h"

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// msg JointState
#include <sensor_msgs/JointState.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace campero_ur_ip_controllers
{
	class JointArrayControl: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			JointArrayControl();
			~JointArrayControl();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			
		private:
		
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // callback function for the command

			ros::Subscriber sub_command_;
			
			realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_; // realtime buffer for the command
			
			int n_joints_; // nb joints of the robot
			
			double publish_rate_; // rate of joint_state topic publishing
			
			ros::Time last_publish_time_; // memorize the 'last time' when the joint_state has been published
			
			std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_; // realtime publisher for the joint_state topic
	};
	
}

#endif


