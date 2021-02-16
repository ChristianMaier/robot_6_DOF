
#include <ros.h>
#include <ArduinoHardware.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include "robot_hardware_interface/pos_service.h"
#include <elapsedMillis.h>
#include <AccelStepper.h>


#define ROS_MESSAGE_CHECK 10000    // 0.01 seconds, times how often arudino is spinned
 
ros::NodeHandle nh;

const int num_joints = 6;

float f_des_position[num_joints];
const int steps_per_rev = 200; // 1,8° per step

int i_des_pos_steps[num_joints];
int des_pos;

AccelStepper stepper1(1, 2, 3);
AccelStepper stepper2(1, 4, 5);
AccelStepper stepper3(1, 6, 7);
AccelStepper stepper4(1, 8, 9);
AccelStepper stepper5(1, 10, 11);
AccelStepper stepper6(1, 12, 13);

AccelStepper joints[num_joints] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};

robot_hardware_interface::pos_service::Response data_exchanger;

// functions-------------- 

void messageCb( const std_msgs::Float32MultiArray& pos_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
  
  for(int i = 0; i < num_joints; i++){
    f_des_position[i] = pos_msg.data[i];

  }
}

void Callback(const robot_hardware_interface::pos_service::Request & req, robot_hardware_interface::pos_service::Response & res)
{ 
  // pos service, delivers information of joint position to controller

  // shape the size of the output message
  res.joint_pos = data_exchanger.joint_pos;
  //res.dim.stride = 1;
  
  for(int i = 0; i < num_joints; i++){
    res.joint_pos[i] = calc_pos_in_rad(joints[i].currentPosition());
  }
}

int calc_pos_in_steps(float pos_in_rad){
  // convert pos from radians to steps
  int steps = round( pos_in_rad/(2*PI)* (float)steps_per_rev);
  return steps;
  }

float calc_pos_in_rad(int pos_in_steps){
  // convert pos from steps to radians
  float pos = ((float)pos_in_steps)/((float)steps_per_rev)*2*PI;
  return pos;
  }

// setup-------------- 

elapsedMicros rosMessageCheck;
ros::Subscriber<std_msgs::Float32MultiArray> sub("ard_des_joint_position", &messageCb );
ros::ServiceServer<robot_hardware_interface::pos_service::Request, robot_hardware_interface::pos_service::Response> service("ard_pos_service", &Callback );

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    nh.initNode();
    nh.advertiseService(service);
    delay(1);
    nh.subscribe(sub);

    // init variables and steppers
    for(int i = 0; i < num_joints; i++){
      f_des_position[i] = 0.0;
      }
    for(int i = 0; i < num_joints; i++){
      i_des_pos_steps[i] = 0.0;
      }
      
    for (int i = 0; i < num_joints; i++){
      joints[i].setMaxSpeed(400);
      joints[i].setAcceleration(200);
    }

    des_pos = 0;
    data_exchanger.joint_pos = (float*)malloc(sizeof(float) * num_joints);

}

// main loop-------------- 

void loop()
{
  // check if the desired position has changed, if yes then set the new position, do this for every joint
  for(int i = 0; i < num_joints; i++){
    
    des_pos = calc_pos_in_steps(f_des_position[i]);
    if(des_pos != i_des_pos_steps[i]){
      joints[i].moveTo(des_pos); 
      i_des_pos_steps[i] = des_pos;
      }
  }

  // run the stepper, also update the position var to the new value
  for(int i = 0; i < num_joints; i++){
    joints[i].run();
  }
  
  // check ros messages, not done in every loop
  if (rosMessageCheck > ROS_MESSAGE_CHECK) {
    nh.spinOnce();
    rosMessageCheck = 0;
  }

}
