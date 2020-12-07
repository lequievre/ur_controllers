/*
 *  A ROS package to control (With an array of joint values) 
 *  the UR10 arm of our campero robotic platform (both simulation and physical robot).
 *  
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
 * This controller can be commanded by a topic 'command', based on std_msgs/Float64MultiArray message :
 * rostopic pub -1 /campero/arm/joint_array_control/command  std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"
 *
 * This controller publish also a 'joint_state' topic, based on sensor_msgs::JointState message.
 * 
 */

#include <joint_array_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace campero_ur_ip_controllers 
{
    JointArrayControl::JointArrayControl(): publish_rate_(0.0), n_joints_(0) {}
    JointArrayControl::~JointArrayControl() {}
    
    bool JointArrayControl::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		ROS_INFO("***** START JointArrayControl::init ************");

		// Initialising KDL chain from reading URDF of the robot
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize JointArrayControl controller.");
            return false;
        }
        
        // get publishing rate (defined in the yaml controller file)
        if (!n.getParam("publish_rate", publish_rate_)) 
        {
			ROS_ERROR("Parameter 'publish_rate' not set");
			return false;
		}
		
		ROS_INFO_STREAM("Publish rate = " << publish_rate_);
        
        // Get nb joints of the robot
        n_joints_ = joint_handles_.size();
        
        // Init commands buffer with initial 0.0 values
        commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
        
        // init realtime publisher for the joint_state topic
		realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "joint_states", 4));

		// get joints name,position,velocity and allocate message of joint_state
		for (unsigned i=0; i<n_joints_; i++){
		  realtime_pub_->msg_.name.push_back(joint_handles_[i].getName());
		  realtime_pub_->msg_.position.push_back(joint_handles_[i].getPosition());
		  realtime_pub_->msg_.velocity.push_back(joint_handles_[i].getVelocity());
		  realtime_pub_->msg_.effort.push_back(joint_handles_[i].getEffort());
		}
        
        // Defining "command" topic, associating to "commandCB" callback function
        sub_command_ = nh_.subscribe("command", 1, &JointArrayControl::commandCB, this);
        
        ROS_INFO("***** FINISH JointArrayControl::init ************");
        
        return true;
	}
    
    void JointArrayControl::starting(const ros::Time& time)
    {
		ROS_INFO("***** START JointArrayControl::starting ************");
		
		// initialize last publish time to calculate the rate
		last_publish_time_ = time;
		
		// initialize a vector of current joint positions
		std::vector<double> current_joints_positions(n_joints_);
		
		// Getting current joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            current_joints_positions[i] = joint_handles_[i].getPosition();
        }
        
        // Init commands buffer with the current joint positions at starting
        commands_buffer_.writeFromNonRT(current_joints_positions);
		
		ROS_INFO("***** FINISH JointArrayControl::starting ************");
	}
	
	void JointArrayControl::stopping(const ros::Time& time)
	{
		ROS_INFO("***** START JointArrayControl::stopping ************");
		
		ROS_INFO("***** FINISH JointArrayControl::stopping ************");
	}
    
    void JointArrayControl::update(const ros::Time& time, const ros::Duration& period)
    {
		// Read commands buffer to send
		std::vector<double> & commands = *commands_buffer_.readFromRT();
		
		// Send joint commands
		for(unsigned int i=0; i<n_joints_; i++)
		{  
			joint_handles_[i].setCommand(commands[i]);  
		}
		
		 // limit rate of publishing
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
        {
			// publish joint state
			if (realtime_pub_->trylock())
			{
					// increment time for publishing
					last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
        
					// populate joint state message:
					realtime_pub_->msg_.header.stamp = time;
					for (unsigned i=0; i<n_joints_; i++){
					  realtime_pub_->msg_.position[i] = joint_handles_[i].getPosition();
					  realtime_pub_->msg_.velocity[i] = joint_handles_[i].getVelocity();
					  realtime_pub_->msg_.effort[i] = joint_handles_[i].getEffort();
					}
					
					realtime_pub_->unlockAndPublish();
			}
		}
	}
	
	void JointArrayControl::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		ROS_INFO("***** START JointArrayControl::command ************");
		
		if ( msg->data.size()!=n_joints_ )
		{ 
			ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
			return; 
		}
		
		commands_buffer_.writeFromNonRT(msg->data);
		
		ROS_INFO("***** FINISH JointArrayControl::command ************");
	}
}

PLUGINLIB_EXPORT_CLASS(campero_ur_ip_controllers::JointArrayControl, controller_interface::ControllerBase)
