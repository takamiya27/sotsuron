#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
// #include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#include <stdint.h>

float joint_position_now[4];
float joint_effort[4];

std_msgs::Float64 rotator_cmd;
std_msgs::Float64 boom_cmd;
std_msgs::Float64 arm_cmd;
std_msgs::Float64 bucket_cmd;

ros::Publisher pub_cmd;

void js_cmd_callback(const sensor_msgs::JointState::ConstPtr& cmd_msg_before){
   sensor_msgs::JointState cmd_msg_after;

   cmd_msg_after.name.resize(8);
   cmd_msg_after.name[0]=cmd_msg_before->name[0];
   cmd_msg_after.name[1]=cmd_msg_before->name[1];
   cmd_msg_after.name[2]=cmd_msg_before->name[2];
   cmd_msg_after.name[3]=cmd_msg_before->name[3];
   cmd_msg_after.name[4]=cmd_msg_before->name[4];
   cmd_msg_after.name[5]=cmd_msg_before->name[5];
   cmd_msg_after.name[6]=cmd_msg_before->name[6];
   cmd_msg_after.name[7]=cmd_msg_before->name[7];

   cmd_msg_after.velocity.resize(8);
   cmd_msg_after.velocity[0] = cmd_msg_before->velocity[0];
   cmd_msg_after.velocity[1] = cmd_msg_before->velocity[1];
   cmd_msg_after.velocity[2] = cmd_msg_before->velocity[2];
   cmd_msg_after.velocity[3] = cmd_msg_before->velocity[3];
   cmd_msg_after.velocity[4] = cmd_msg_before->velocity[4];
   cmd_msg_after.velocity[5] = cmd_msg_before->velocity[5];
   cmd_msg_after.velocity[6] = cmd_msg_before->velocity[6];
   cmd_msg_after.velocity[7] = cmd_msg_before->velocity[7];

   cmd_msg_after.effort.resize(8);
   cmd_msg_after.effort[0] = cmd_msg_before->effort[0];
   cmd_msg_after.effort[1] = cmd_msg_before->effort[1];
   cmd_msg_after.effort[2] = cmd_msg_before->effort[2];
   cmd_msg_after.effort[3] = cmd_msg_before->effort[3];
   cmd_msg_after.effort[4] = cmd_msg_before->effort[4];
   cmd_msg_after.effort[5] = cmd_msg_before->effort[5];
   cmd_msg_after.effort[6] = cmd_msg_before->effort[6];
   cmd_msg_after.effort[7] = cmd_msg_before->effort[7];

   pub_cmd.publish(cmd_msg_after);
}

int main(int argc, char**argv){
   ros::init(argc,argv,"cmd_coordinator_warning_sound");
   ros::NodeHandle nh("cmd_coordinator_warning_sound");

   ros::Subscriber sub_cmd = nh.subscribe("js_cmd_before", 100, js_cmd_callback);

   pub_cmd = nh.advertise<sensor_msgs::JointState>("js_cmd_after",100);

   ros::spin();

   return 0;
}