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
 */

#include <cartesian_velocity_control.h>

// Utils for pseudo inverse and skew_symmetric
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

// For plugin
#include <pluginlib/class_list_macros.h>


namespace campero_ur_ip_controllers 
{
    CartesianVelocityControl::CartesianVelocityControl() {}
    CartesianVelocityControl::~CartesianVelocityControl() {}

    bool CartesianVelocityControl::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		ROS_INFO("***** START CartesianVelocityControl::init ************");

		// Initialising KDL chain from reading URDF of the robot
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize CartesianVelocityControl controller.");
            return false;
        }

		// Initialising KDL Solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

		// Resizing Jacobian matrix
        J_.resize(kdl_chain_.getNrOfJoints());

		// Defining command topic
        sub_command_ = nh_.subscribe("command", 1, &CartesianVelocityControl::command, this);

		// current cartesian publisher
		realtime_x_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(n, "current_x", 4));
		
		#if PUBLISH_MARKERS
		 // current cartesian position into a marker publisher, can be added in RVIZ	
		 realtime_marker_x_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>(n, "marker_x", 4));
		#endif
        
        ROS_INFO("***** FINISH CartesianVelocityControl::init ************");

        return true;
    }

    void CartesianVelocityControl::starting(const ros::Time& time)
    {
		ROS_INFO("***** CartesianVelocityControl::starting ************");

		// Initialising the current time (for debug)
		last_time_ = ros::Time::now();
		last_time_error_rot_ = ros::Time::now();
		
		// Getting current joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

		// Computing current cartesian position by using the forward kinematic solver
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // Desired posture is the current one
        x_des_buffer_.writeFromNonRT(x_);

		// Setting command flag to 0 to not run the update method
        cmd_flag_ = 0;

		// Setting on target to 0 to specify that the effector is not on desired target
        on_target_ = 0;
        
        #if PUBLISH_MARKERS
         msg_id_ = 0; // Set the default value of a marker array message id to 0.
         realtime_marker_x_pub_->msg_.markers.resize(1); // The array of the message array contain only one marker.
        #endif
    }

	void CartesianVelocityControl::stopping(const ros::Time& time)
	{
		ROS_INFO("***** CartesianVelocityControl::stopping ************");
		
		// No command
		cmd_flag_ = 0;

		// Not on target
		on_target_ = 0;
	}

    void CartesianVelocityControl::update(const ros::Time& time, const ros::Duration& period)
    {
		//ROS_INFO("***** CartesianVelocityControl::update start ************");
		
        // Getting current joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }
        
        // Computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        

		#if TRACE_ACTIVATED
		
			// Debugging code to see if the current cartesian pose is closed to current pose
		
			current_time_ = ros::Time::now();
			ros::Duration elapsed_time = current_time_ - last_time_;

			// Each 1.O second
			if (elapsed_time.toSec() >= 1.0)
			{
				if (!on_target_)
				{
					for(int i=0; i<joint_handles_.size(); ++i)
					{
						ROS_INFO("q[%d]=%f", i, joint_msr_states_.q(i));
					}

					ROS_INFO_STREAM("Current cartesian position x= " << x_.p.x() << ", y= " << x_.p.y() << ", z= "<< x_.p.z());
					x_.M.GetRPY(x_roll_,x_pitch_,x_yaw_);
					ROS_INFO_STREAM("Current cartesian orientation roll= " << x_roll_ << ", pitch= " << x_pitch_ << ", yaw= "<< x_yaw_);
					
					x_des_ = x_des_buffer_.readFromRT();
					x_des_->M.GetRPY(x_roll_des_,x_pitch_des_,x_yaw_des_);
					ROS_INFO_STREAM("orientation diff roll= " << (x_roll_des_-x_roll_) << ", pitch= " << (x_pitch_des_-x_pitch_) << ", yaw= "<< (x_yaw_des_-x_yaw_));
					
					KDL::Vector euc_dist = x_des_->p - x_.p;
					double norm_euc_dist = euc_dist.Norm();
					ROS_INFO_STREAM("euclidean distance = " << norm_euc_dist);
				}
				   
			    last_time_ = current_time_;
			}

		#endif

        if (cmd_flag_)
        {
			// Reading realtime buffer (X desired pose) without to be blocked.
			x_des_ = x_des_buffer_.readFromRT();
			
            // Computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // Computing J_pinv_ (Jacobian pseudo inverse)
            pseudo_inverse(J_.data, J_pinv_);

			// Reset position and orientation error
			SetToZero(x_err_);
			
            // End-effector position (velocity) error
            x_err_.vel = x_des_->p - x_.p; // pay attention x_des is now a velocity in m/s 

            // Getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_->M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // End-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;
           
            // Computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
            }

            // Integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // Joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }


			// Test if position and orientation are closed to desired pose (0.001 m for position, 0.001 rad for each RPY angles)
            if (Equal(x_, *x_des_, 0.001))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;
				on_target_ = 1;
            }
            
           /* 
            ROS_INFO("update position desired -> x = %f, y = %f, z = %f", x_des_.p.x(), x_des_.p.y(), x_des_.p.z());
            ROS_INFO("update position calculated -> x = %f, y = %f, z = %f", x_.p.x(), x_.p.y(), x_.p.z());
            ROS_INFO("error translation (des-cal) -> x = %f, y = %f, z = %f", x_err_.vel.x(), x_err_.vel.y(), x_err_.vel.z());
            ROS_INFO("update rotation calculated ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_.M.data[0],x_.M.data[1],x_.M.data[2],x_.M.data[3],x_.M.data[4],x_.M.data[5],x_.M.data[6],x_.M.data[7],x_.M.data[8]);
            ROS_INFO("update rotation desired ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_des_.M.data[0],x_des_.M.data[1],x_des_.M.data[2],x_des_.M.data[3],x_des_.M.data[4],x_des_.M.data[5],x_des_.M.data[6],x_des_.M.data[7],x_des_.M.data[8]);
           */

		   // set command to joints
           for (int i = 0; i < joint_handles_.size(); i++)
           {
             joint_handles_[i].setCommand(joint_des_states_.q(i));
           }

        }
        
        #if PUBLISH_MARKERS
         // setting marker parameters
		 publish_marker_x_(x_, msg_id_);
		#endif
        
        // publish estimated cartesian pose
		if (realtime_x_pub_->trylock()){
			tf::poseKDLToMsg(x_,realtime_x_pub_->msg_);
			 realtime_x_pub_->unlockAndPublish();
		}

	    //ROS_INFO("***** CartesianVelocityControl::update end ************");
    }

    void CartesianVelocityControl::command(const campero_ur_ip_controllers::PoseRPY::ConstPtr &msg)
    {
		ROS_INFO("***** START OneTaskInverseKinematics::command ************");

        KDL::Frame frame_des_;

        // Computing current cartesian position by using the forward kinematic solver
        fk_pos_solver_->JntToCart(joint_msr_states_.q, frame_des_);

        switch(msg->id)
        {
            case 0:  // position and orientation
				ROS_INFO("***** CartesianVelocityControl::command position and orientation ************");
				ROS_INFO("position desired -> x = %f, y = %f, z = %f", msg->position.x, msg->position.y, msg->position.z);
				ROS_INFO("orientation desired -> roll = %f, pitch = %f, yaw = %f", msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw);
				frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
				ROS_INFO("***** CartesianVelocityControl::command position only ************");
				ROS_INFO("position desired -> x = %f, y = %f, z = %f", msg->position.x, msg->position.y, msg->position.z);
				frame_des_.p = KDL::Vector(msg->position.x, msg->position.y, msg->position.z);
            break;

            case 2: // orientation only
				ROS_INFO("***** CartesianVelocityControl::command orientation only ************");
				ROS_INFO("orientation desired -> roll = %f, pitch = %f, yaw = %f", msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw);

				frame_des_.M = KDL::Rotation::RPY(msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw);
            break;

            default:
				ROS_INFO("Wrong message ID !  must be 0 or 1 or 2 !");
            return;
        }
        
        // Using realtime buffer (writing Non RT)
		x_des_buffer_.writeFromNonRT(frame_des_);

        cmd_flag_ = 1;
		on_target_ = 0;
		
		#if PUBLISH_MARKERS
		 msg_id_ = 0;
		#endif
        
        ROS_INFO("***** FINISH CartesianVelocityControl::command ************");
    }
    
   #if PUBLISH_MARKERS
    void CartesianVelocityControl::publish_marker_x_(KDL::Frame x, int id)
	{		
		if (realtime_marker_x_pub_->trylock()){
			    
			    int index = 0;
			    msg_id_++;
			    

				realtime_marker_x_pub_->msg_.markers[index].header.frame_id = "campero_ur10_base_link";
				realtime_marker_x_pub_->msg_.markers[index].header.stamp = ros::Time();
				realtime_marker_x_pub_->msg_.markers[index].ns = "end_effector";
				realtime_marker_x_pub_->msg_.markers[index].id = id;
				realtime_marker_x_pub_->msg_.markers[index].type = visualization_msgs::Marker::SPHERE;
				realtime_marker_x_pub_->msg_.markers[index].action = visualization_msgs::Marker::ADD;
				realtime_marker_x_pub_->msg_.markers[index].pose.position.x = x.p.x();
				realtime_marker_x_pub_->msg_.markers[index].pose.position.y = x.p.y();
				realtime_marker_x_pub_->msg_.markers[index].pose.position.z = x.p.z();
				
				x.M.GetQuaternion(
				realtime_marker_x_pub_->msg_.markers[index].pose.orientation.x,
				realtime_marker_x_pub_->msg_.markers[index].pose.orientation.y,
				realtime_marker_x_pub_->msg_.markers[index].pose.orientation.z,
				realtime_marker_x_pub_->msg_.markers[index].pose.orientation.w);
				
				realtime_marker_x_pub_->msg_.markers[index].scale.x = 0.01;
				realtime_marker_x_pub_->msg_.markers[index].scale.y = 0.01;
				realtime_marker_x_pub_->msg_.markers[index].scale.z = 0.01;
				realtime_marker_x_pub_->msg_.markers[index].color.a = 1.0;
				realtime_marker_x_pub_->msg_.markers[index].color.r = 0.0;
				realtime_marker_x_pub_->msg_.markers[index].color.g = 1.0;
				realtime_marker_x_pub_->msg_.markers[index].color.b = 0.0;
				
				
				realtime_marker_x_pub_->unlockAndPublish();
			}
			
	}
  #endif
}

PLUGINLIB_EXPORT_CLASS(campero_ur_ip_controllers::CartesianVelocityControl, controller_interface::ControllerBase)

