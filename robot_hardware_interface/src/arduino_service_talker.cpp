//
// Created by christian on 01.11.21.
//

#include <robot_hardware_interface/robot_hardware_interface.h>
#include <robot_hardware_interface/pos_service.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

static int num_joints = 6; // TODO make this interchangeable for every type of robot
float joint_position_ard[6] = { 0.0 };

bool serviceCallback(robot_hardware_interface::pos_service::Request &req, robot_hardware_interface::pos_service::Response &res)
{
    // callback of position_service. Response are the last information from the arduino

    // resize the msg vector, else data size is unclear
    res.joint_pos.data.resize(num_joints);
    for(int i = 0; i < num_joints; i++){
        res.joint_pos.data[i] = joint_position_ard[i];
    }
    return true;
}

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i = 0; i < num_joints; i++){
        joint_position_ard[i]= msg->data[i];
    }
}

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "arduino_talker_service");

   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("/ard_cur_joint_position", 1000, chatterCallback);
   ros::ServiceServer service = n.advertiseService("/ard_pos_service", serviceCallback);

   ros::spin();

   return 0;
}
