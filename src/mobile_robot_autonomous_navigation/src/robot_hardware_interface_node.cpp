#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>

//namesapce i2c_ros

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
  
 for (int i = 0; i < 4; i++){
     joint_positions_[i] = 0.0;
        joint_velocities_[i] = 0.0;
        joint_efforts_[i] = 0.0;
        joint_velocity_commands_[i] = 0.0;
    }

    joint_names_[0] = "front_left_wheel_joint";
    joint_names_[1] = "front_right_wheel_joint";
    joint_names_[2] = "back_left_wheel_joint";
    joint_names_[3] = "back_right_wheel_joint";

	for(int i=0; i<4; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_commands_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {
    uint8_t rbuff[1];
    int x;
   
   for (int i =0; i < 4; ++i){
}
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

	uint8_t wbuff[2];

    int velocity,result;
    
    for (int i = 0; i < 4; ++i) {
    velocity=(int)angles::to_degrees(joint_velocity_commands_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

        if (i == 0 && front_left_prev_cmd != velocity) {
            // You need to replace these lines with your actual motor writing operations
            // result = left_motor.writeData(wbuff, 2);
            // left_prev_cmd = velocity;
        } else if (i == 1 && front_right_prev_cmd != velocity) {
            // You need to replace these lines with your actual motor writing operations
            // result = right_motor.writeData(wbuff, 2);
            // right_prev_cmd = velocity;
        } else if (i == 2 && back_left_prev_cmd != velocity) {
            // You need to replace these lines with your actual motor writing operations
            // result = back_left_motor.writeData(wbuff, 2);
            // back_left_prev_cmd = velocity;
        } else if (i == 3 && back_right_prev_cmd != velocity) {
            // You need to replace these lines with your actual motor writing operations
            // result = back_right_motor.writeData(wbuff, 2);
            // back_right_prev_cmd = velocity;
        }
}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
