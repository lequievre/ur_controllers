/*
 *  A ROS package to control the KUKA LWR 4 (both simulation and physical robot). 
 *  This package was originally forked from CentroEPiaggio/kuka-lwr.
 *  <author>Manuel Bonilla</author>
 *  <maintainer email="cjrosales@gmail.com">Carlos J. Rosales</maintainer>
 *  <maintainer email="marco.esposito@tum.de">Marco Esposito</maintainer>
 *  <license>BSD</license>
 *  
 *  This package was adapted for Universal Robot UR10 by :
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
 * rostopic pub -1 /campero/arm/cartesian_velocity_control/command  campero_ur_ip_controllers/PoseRPY '{id: 0, position: {x: 0.5, y: 0.5, z: 0.8}, orientation: {roll: 1.8, pitch: -0.3, yaw: 1.3}}'
 *
 * 
 */

#ifndef LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H

// Controller base
#include "kinematic_chain_controller_base.h"

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl_conversions/kdl_msg.h>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// Ros messages generated
#include <campero_ur_ip_controllers/PoseRPY.h>
#include <geometry_msgs/Pose.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#define NB_PRINTED_ON_TARGET_MAX 1

namespace campero_ur_ip_controllers
{
	class CartesianVelocityControl: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			CartesianVelocityControl();
			~CartesianVelocityControl();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void command(const campero_ur_ip_controllers::PoseRPY::ConstPtr &msg);

		private:
			ros::Subscriber sub_command_;

			KDL::Frame x_;		// current pose
			KDL::Frame* x_des_;	// desired pose
			realtime_tools::RealtimeBuffer<KDL::Frame> x_des_buffer_; // real time buffer for setting and reading desired pose 
			
			double x_roll_, x_pitch_, x_yaw_;  // current RPY orientation
			double x_roll_des_, x_pitch_des_, x_yaw_des_; // desired RPY orientation

			KDL::Twist x_err_;  // position and velocity error

			KDL::Jacobian J_;	// Jacobian

			Eigen::MatrixXd J_pinv_;  // Jacobian inversed
			Eigen::Matrix<double,3,3> skew_;

			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_x_pub_; // real time publisher to publish current cartesian position and orientation

			struct quaternion_
			{
				KDL::Vector v;
				double a;
			} quat_curr_, quat_des_;  // Structure defining a quaternion

			KDL::Vector v_temp_;
	
			int cmd_flag_; // flag setted when a command is send from the command topic (a desired pose)
			
			int on_target_; // flag setted when the current pose of the robot is near from desired pose
	
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;  // a solver to calculate the jacobian of a KDL::Chain
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; // the forward position kinematics solver for a KDL::Chain
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;  // the inverse velocity solver for a KDL::Chain
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_; // the inverse position solver for a KDL::Chain
			
			ros::Time  last_time_, current_time_; // time for debug
			ros::Time  last_time_error_rot_, current_time_error_rot_; // time for debug
	};
}

#endif
