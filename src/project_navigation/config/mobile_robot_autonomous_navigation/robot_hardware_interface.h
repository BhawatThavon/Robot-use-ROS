#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>

#include <mobile_robot_autonomous_navigation/i2c_ros.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        ros::Publisher pub;
        ros::ServiceClient client;
        rospy_tutorials::Floats joints_pub;
        //three_dof_planar_manipulator::Floats_array joint_read;
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        
        std::string joint_names_[4]={"front_left_wheel_joint","front_right_wheel_joint","back_left_wheel_joint","back_right_wheel_joint"};  
        double joint_positions_[4];
        double joint_velocities_[4];
        double joint_efforts_[4];
        double joint_velocity_commands_[4];


	
	double left_motor_pos=0,right_motor_pos=0;
        int front_left_prev_cmd=0, front_right_prev_cmd=0;
        int back_left_prev_cmd=0, back_right_prev_cmd=0;

       	i2c_ros::I2C front_left_motor= i2c_ros::I2C(0, 0x08);
        i2c_ros::I2C front_right_motor= i2c_ros::I2C(1, 0x09);
       	i2c_ros::I2C back_left_motor= i2c_ros::I2C(2, 0x0A);
        i2c_ros::I2C back_right_motor= i2c_ros::I2C(3, 0x0B);


        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

