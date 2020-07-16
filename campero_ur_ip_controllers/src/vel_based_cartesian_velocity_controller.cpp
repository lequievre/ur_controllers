/*
 *  A ROS controller to drive the robot with cartesian velocity commands
 *  VelocityJointInterface is used for moving the robot
 * 
 *  A. Koessler, L. Lequièvre
 *  Institut Pascal, Université Clermont Auvergne, CNRS, SIGMA Clermont
 * 
 *  adrien.koessler(at)sigma-clermont.fr, laurent lequievre(at)uca.fr
 * 
 *  Last updated 2020-07-15
*/

#include <vel_based_cartesian_velocity_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>


namespace mrod_ur_ip_controllers 
{
    VelBasedCartesianVelocityController::VelBasedCartesianVelocityController() {}
    VelBasedCartesianVelocityController::~VelBasedCartesianVelocityController() {}
    
    bool VelBasedCartesianVelocityController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
    {
		ROS_INFO("***** START VelBasedCartesianVelocityController::init ************");

        if( !(KinematicChainControllerBase<hardware_interface::VelocityJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize VelBasedCartesianVelocityController controller.");
            return false;
        }

        // kinematic solvers setup
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        q_dot_cmd_.resize(kdl_chain_.getNrOfJoints());
       
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }
        
        // subscriber to desired cartesian velocity topic
        sub_command_ = nh_.subscribe("cmd_vel", 1, &VelBasedCartesianVelocityController::command, this);
        
        // publishers for current cartesian pose and velocity
        realtime_x_dot_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(n, "x_dot_master", 4));
        realtime_x_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(n, "x_master", 4));
        
        ROS_INFO("***** FINISH VelBasedCartesianVelocityController::init ************");

        return true;
    }

    void VelBasedCartesianVelocityController::starting(const ros::Time& time)
    {
		ROS_INFO("***** CartesianVelocityControl::starting ************");

		// Initialising the current time (for debug)
		last_time_ = ros::Time::now();
		
		// Computing current cartesian position by using the forward kinematic solver
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        
        // Fill articular values with current measurements
        q_dot_current_.q.resize(kdl_chain_.getNrOfJoints());
        q_dot_current_.qdot.resize(kdl_chain_.getNrOfJoints());
        q_dot_current_.q = joint_msr_states_.q;
        q_dot_current_.qdot = joint_msr_states_.qdot;
        
        // Compute current cartesian velocity through FK solver
        fk_vel_solver_->JntToCart(q_dot_current_, x_frame_dot_current_);
        

        // Fill desired cartesian and articular values with 0 for safety
        SetToZero(x_dot_0);
        SetToZero(q_dot_cmd_);	
        x_dot_des_buffer_.writeFromNonRT(x_dot_0);

		// Setting flag command to 0 to not run the update method
        cmd_flag_ = 0;
    }

	void VelBasedCartesianVelocityController::stopping(const ros::Time& time)
	{
		ROS_INFO("***** VelBasedCartesianVelocityController::stopping ************");
		
		cmd_flag_ = 0;

	}

    void VelBasedCartesianVelocityController::update(const ros::Time& time, const ros::Duration& period)
    {
		//ROS_INFO("***** CartesianVelocityControl::update debut ************");
		
        // Get current joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        }

       
		// Read realtime buffer without to be blocked.
		x_dot_des_ = x_dot_des_buffer_.readFromRT();
		
		// compute forward kinematics
		fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
		
		// get current articular values
		q_dot_current_.q = joint_msr_states_.q;
		q_dot_current_.qdot = joint_msr_states_.qdot;
		
		// compute current cartesian velocity
		fk_vel_solver_->JntToCart(q_dot_current_, x_frame_dot_current_);
		
		// run IK solver to compute target articular velocity based on desired cartesian velocity
		ik_vel_solver_->CartToJnt(q_dot_current_.q, *x_dot_des_, q_dot_cmd_);
		

		// set command to joints
		for (int i = 0; i < joint_handles_.size(); i++)
		{
		 joint_handles_[i].setCommand(q_dot_cmd_.data(i));
		}
            
        // publish estimated cartesian pose and velocity
        if (realtime_x_dot_pub_->trylock()){
			tf::twistKDLToMsg(x_frame_dot_current_.GetTwist(),realtime_x_dot_pub_->msg_);
			realtime_x_dot_pub_->unlockAndPublish();
		}
		if (realtime_x_pub_->trylock()){
			tf::poseKDLToMsg(x_,realtime_x_pub_->msg_);
			 realtime_x_pub_->unlockAndPublish();
		}
        
        #if TRACE_ACTIVATED // prints (define TRACE_ACTIVATED as 0 to suppress)
			current_time_ = ros::Time::now();
			ros::Duration elapsed_time = current_time_ - last_time_;
			if (elapsed_time.toSec() >= 1.0)
			{
				for (int i = 0; i < joint_handles_.size(); i++)
                {
                  ROS_INFO_STREAM("name = " << kdl_chain_.getSegment(i).getJoint().getName() << ", command = " << q_dot_cmd_.data(i));
                }   
			    last_time_ = current_time_;
			}
		#endif


    }

    void VelBasedCartesianVelocityController::command(const geometry_msgs::Twist::ConstPtr &msg)
    {
		ROS_INFO("***** START OneTaskInverseKinematics::command ************");
		
		// convert topic type (geometry_msgs::Twist) to type used in update method (KDL::Twist)
        KDL::Twist x_dot_des_;
        tf::TwistMsgToKDL(*msg, x_dot_des_);
     
        // Using realtime buffer (writing Non RT)
		x_dot_des_buffer_.writeFromNonRT(x_dot_des_);

        
        ROS_INFO("***** FINISH VelBasedCartesianVelocityController::command ************");
    }
    
    
}

PLUGINLIB_EXPORT_CLASS(mrod_ur_ip_controllers::VelBasedCartesianVelocityController, controller_interface::ControllerBase)
