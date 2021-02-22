#include <robot_hardware_interface/robot_hardware_interface.h>
#include <robot_hardware_interface/pos_service.h>
#include <unistd.h>



ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10.0;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<std_msgs::Float32MultiArray>("/ard_des_joint_position",10);
	client = nh_.serviceClient<robot_hardware_interface::pos_service>("/ard_pos_service");

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {

    // initialize interfaces for robot control
    num_joints_ = 6;
    joint_names_.resize(num_joints_);
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);

    joints_pub.data.resize(num_joints_);

    trans_ratio.push_back(7.1);
    trans_ratio.push_back(10.0);
    trans_ratio.push_back(10.0);
    trans_ratio.push_back(10.0);
    trans_ratio.push_back(10.0);
    trans_ratio.push_back(10.0);

    std::stringstream ss;
    for (int i=0; i < num_joints_; i++) {
        joint_names_[i] = "Joint_";
        joint_names_[i].append(std::to_string(i+1));
        //std::cout << joint_names_[i];
        ss << "Hi, joint name is: " << joint_names_[i];
        ROS_INFO("%s\n", ss.str().c_str() );
        ss.str("");

// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

// Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
    position_joint_interface_.registerHandle(jointPositionHandle);
    
// Create velocity joint interface
	//hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    //effort_joint_interface_.registerHandle(jointVelocityHandle);

	
// Create Joint Limit interface

//    joint_limits_interface::JointLimits limits;
//    joint_limits_interface::getJointLimits("Joint_1", nh_, limits);
//	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
//	effortJointSaturationInterface.registerHandle(jointLimitsHandle);
    }

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    ROS_INFO("Registering Joints done.");

    // initialize transmission interfaces

    /*
    SimpleTransmission trans_1(10.0);
    SimpleTransmission trans_2(10.0);
    SimpleTransmission trans_3(10.0);
    SimpleTransmission trans_4(10.0);
    SimpleTransmission trans_5(10.0);
    SimpleTransmission trans_6(10.0);
    SimpleTransmission sim_trans[6] = {trans_1,trans_2,trans_3,trans_4,trans_5,trans_6};
    for(int i=0; i < num_joints_; i++) {

        // Wrap simple transmission raw data - current state
        a_state_data[i].position.push_back(&a_curr_pos[i]);// Velocity and effort vectors are unused
        j_state_data[i].position.push_back(&j_curr_pos[i]);// Velocity and effort vectors are unused
        // Wrap simple transmission raw data - position command
        a_cmd_data[i].position.push_back(&a_cmd_pos[i]); // Velocity and effort vectors are unused
        j_cmd_data[i].position.push_back(&j_cmd_pos[i]); // Velocity and effort vectors are unused

        // register handles
        std::string trans_name = "trans_";
        trans_name.append(std::to_string(i+1));
        act_to_jnt_state.registerHandle(ActuatorToJointStateHandle(trans_name,
                                                                 &sim_trans[i] ,
                                                                 a_state_data[i],
                                                                 j_state_data[i]));
        jnt_to_act_pos.registerHandle(JointToActuatorPositionHandle(trans_name,
                                                                    &sim_trans[i],
                                                                    a_cmd_data[i],
                                                                    j_cmd_data[i]));
    }
    for (int i=0; i < num_joints_; i++){
        a_curr_pos[i] = 0.0;
        a_cmd_pos[i] = 0.0;
        j_curr_pos[i] = 0.0;
        j_cmd_pos[i] = 0.0;

    }
    ROS_INFO("Successfully created transmission interfaces.");
    */
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {

    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {
    robot_hardware_interface::pos_service srv;

    if(client.call(srv))
	{
        for (int i=0; i < srv.response.joint_pos.data.size(); i++) {
            joint_position_[i] = (srv.response.joint_pos.data[i] / trans_ratio[i] );
            joint_velocity_[i] = 0.0;
        }
        //act_to_jnt_state.propagate();
	    ROS_INFO("Current Pos: %.2f, Vel: %.2f",joint_position_[0],joint_velocity_[0]);
	}
	else
	{
        for (int i=0; i < num_joints_; i++) {
            joint_position_[i] = 0.0;
            joint_velocity_[i] = 0.0;
        }
	}
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {

    effortJointSaturationInterface.enforceLimits(elapsed_time);
    //jnt_to_act_pos.propagate();
    for (int i=0; i < num_joints_; i++) {
        joints_pub.data[i] = (float) (joint_position_command_[i]*trans_ratio[i]);

    }
    ROS_INFO("Position Cmd: %.2f", joint_position_command_[0]);
	pub.publish(joints_pub);
		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);

    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
