/*
 *  A ROS package to control ......
 *  Adrien KOESSLER
 *  adrien.koessler@sigma-clermont.fr
 *  
 * 
*/


#ifndef MROD_CONTROLLERS__H
#define MROD_CONTROLLERS__H

// Controller base
#include "kinematic_chain_controller_base.h"

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <kdl_conversions/kdl_msg.h>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Float 64 MultiArray message
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

namespace campero_ur_ip_controllers
{
	class VelBasedCartesianVelocityController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::VelocityJointInterface>
	{
		public:
			VelBasedCartesianVelocityController();
			~VelBasedCartesianVelocityController();

			bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void command(const geometry_msgs::Twist::ConstPtr &msg);

		private:
			ros::Subscriber sub_command_;
			int cmd_flag_;
			
			realtime_tools::RealtimeBuffer<KDL::Twist> x_dot_des_buffer_;
			
			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > realtime_x_dot_pub_;
			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_x_pub_;
				
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
			
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			
			KDL::JntArray q_dot_cmd_; // computed set points
			
			ros::Time  last_time_, current_time_;
			
			KDL::Twist x_dot_0;
			KDL::Frame x_;		//current pose
			KDL::JntArrayVel q_dot_current_;
			
			KDL::FrameVel x_frame_dot_current_;
			
			KDL::Twist* x_dot_des_;	//desired pose
		
	};

}

#endif
