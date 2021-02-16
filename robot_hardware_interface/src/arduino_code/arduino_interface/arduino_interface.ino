
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

float f_position = 0.0;
float f_des_position[num_joints];
const int steps_per_rev = 200; // 1,8° per step


AccelStepper stepper1(1, 2, 3);
int i_des_pos_steps = 0;

// functions-------------- 

void messageCb( const std_msgs::Float32MultiArray& pos_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
  for(int i = 0; i < num_joints; i++){
    f_des_position[i] = pos_msg.data[i];

  }
}

void Callback(const robot_hardware_interface::pos_service::Request & req, robot_hardware_interface::pos_service::Response & res)
{
   res.joint_pos[0] = f_position;
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
ros::Subscriber<std_msgs::Float32MultiArray> sub("ard_des_joint_position_1", &messageCb );
ros::ServiceServer<robot_hardware_interface::pos_service::Request, robot_hardware_interface::pos_service::Response> service("ard_pos_service", &Callback );

void setup()
  {
    pinMode(LED_BUILTIN, OUTPUT);
    nh.initNode();
    nh.advertiseService(service);
    delay(1);
    nh.subscribe(sub);

    for(int i = 0; i < num_joints; i++){
      f_des_position[i] = 0.0;
      }

    stepper1.setMaxSpeed(400);
    stepper1.setAcceleration(200);

    }

// main loop-------------- 

void loop()
{
  // check if the desired position has changed, if yes then set the new position
  int des_pos = calc_pos_in_steps(f_des_position[0]);
  if(des_pos != i_des_pos_steps){
    stepper1.moveTo(des_pos); 
    i_des_pos_steps = des_pos;
    }

  // run the stepper, also update the position var to the new value
  stepper1.run();
  f_position = calc_pos_in_rad(stepper1.currentPosition());
  
  // check ros messages, not done in every loop
  if (rosMessageCheck > ROS_MESSAGE_CHECK) {
    nh.spinOnce();
    rosMessageCheck = 0;
  }

}
