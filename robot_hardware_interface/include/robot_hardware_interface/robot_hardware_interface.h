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
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

using namespace transmission_interface;

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
        std_msgs::Float32MultiArray joints_pub;
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::EffortJointInterface     effort_joint_interface_;

        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        
        int                                          num_joints_;
        std::vector<std::string>                     joint_names_;
        std::vector<double>                          joint_position_;
        std::vector<double>                          joint_velocity_;
        std::vector<double>                          joint_effort_;
        std::vector<double>                          joint_position_command_;
        std::vector<double>                          joint_velocity_command_;

        std::vector<double>                          joint_lower_limits_;
        std::vector<double>                          joint_upper_limits_;

        // interfaces for transmission, TODO adjust for different number of joints
        std::vector<double>                          trans_ratio;
        /*
        ActuatorToJointStateInterface       act_to_jnt_state; // For propagating current actuator state to joint space
        JointToActuatorPositionInterface    jnt_to_act_pos;
        //SimpleTransmission                  sim_trans[6];

        ActuatorData                        a_state_data[6]; // Size 2: One per transmission
        ActuatorData                        a_cmd_data[6];

        JointData                           j_state_data[6];
        JointData                           j_cmd_data[6];

        double a_curr_pos[6]; // Size 3: One per actuator
        double a_cmd_pos[6];
        double j_curr_pos[6]; // Size 3: One per joint
        double j_cmd_pos[6];
        */

        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

