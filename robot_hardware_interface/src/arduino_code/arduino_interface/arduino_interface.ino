
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

// variables -------------
 
ros::NodeHandle nh;

const int num_joints = 6;

float f_des_position[num_joints];
const int steps_per_rev[num_joints] = {400, 1600, 400, 400, 400, 400};  // different for each motor because of different microstepping
const int max_speed[num_joints] = {800, 3200, 800, 800, 800, 800};       // different for each motor because of different microstepping
const int max_acceleration[num_joints] = {200, 400, 200, 200, 200, 200};

int i_des_pos_steps[num_joints];
int des_pos;

AccelStepper stepper1(1, 2, 3);
AccelStepper stepper2(1, 12, 13);
AccelStepper stepper3(1, 6, 7);
AccelStepper stepper4(1, 8, 9);
AccelStepper stepper5(1, 10, 11);
AccelStepper stepper6(1, 4, 5);

AccelStepper joints[num_joints] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};

std_msgs::Float32MultiArray data_exchanger;
ros::Publisher pub("/ard_cur_joint_position", &data_exchanger);

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
  //res.joint_pos = (float*)malloc(sizeof(float) * num_joints);//data_exchanger.joint_pos;
  //res.dim.stride = 1;
  res.joint_pos = data_exchanger;
  //res.joint_pos.data_length = num_joints;
  
  for(int i = 0; i < num_joints; i++){
    res.joint_pos.data[i] = calc_pos_in_rad(joints[i].currentPosition(), i);
  }
}

void publish_data(){
  // function gets the data from the current joint position and publish it to the topic
  
  for(int i = 0; i < num_joints; i++){
    data_exchanger.data[i] = calc_pos_in_rad(joints[i].currentPosition(), i);
  }

  pub.publish( &data_exchanger );
  
}

int calc_pos_in_steps(float pos_in_rad, int joint_number){
  // convert pos from radians to steps
  int steps = round( pos_in_rad/(2*PI)* (float)steps_per_rev[joint_number]);
  return steps;
  }

float calc_pos_in_rad(int pos_in_steps, int joint_number){
  // convert pos from steps to radians
  float pos = ((float)pos_in_steps)/((float)steps_per_rev[joint_number])*2*PI;
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
    //nh.advertiseService(service);   ------------------------------ service deactivated
    delay(1);
    nh.subscribe(sub);
    delay(1);
    nh.advertise(pub);   

    // init variables and steppers
    for(int i = 0; i < num_joints; i++){
      f_des_position[i] = 0.0;
      }
    for(int i = 0; i < num_joints; i++){
      i_des_pos_steps[i] = 0.0;
      }
      
    for (int i = 0; i < num_joints; i++){
      joints[i].setMaxSpeed(max_speed[i]);
      joints[i].setAcceleration(max_acceleration[i]);
    }

    des_pos = 0;
    data_exchanger.data = (float*)malloc(sizeof(float) * num_joints);
    data_exchanger.data_length = num_joints;

}

// main loop-------------- 

void loop()
{
  // check if the desired position has changed, if yes then set the new position, do this for every joint
  for(int i = 0; i < num_joints; i++){
    
    des_pos = calc_pos_in_steps(f_des_position[i], i);
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
    publish_data();
    nh.spinOnce();
    rosMessageCheck = 0;
  }

}
