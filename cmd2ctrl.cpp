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

void js_cmd_callback(const sensor_msgs::JointState::ConstPtr& cmd_msg){
   // ROS_INFO("sub cmd!");

   rotator_cmd.data = cmd_msg->velocity[0]; //rotator
   boom_cmd.data = cmd_msg->velocity[1]; //boom
   arm_cmd.data = cmd_msg->velocity[2]; //arm
   bucket_cmd.data = cmd_msg->velocity[3]; //bucket

   // ROS_INFO("effort: %f, %f, %f, %f", joint_effort[0], joint_effort[1], joint_effort[2], joint_effort[3]); 
}

int main(int argc, char**argv){
   ros::init(argc,argv,"cmd2ctrl");
   ros::NodeHandle nh("cmd2ctrl");

   ros::Subscriber sub_cmd = nh.subscribe("js_cmd", 100, js_cmd_callback);

   ros::Publisher pub_rotator_ctrl = nh.advertise<std_msgs::Float64>("zx135u/rotator_joint_controller/command",100);
   ros::Publisher pub_boom_ctrl = nh.advertise<std_msgs::Float64>("zx135u/boom_joint_controller/command",100);
   ros::Publisher pub_arm_ctrl = nh.advertise<std_msgs::Float64>("zx135u/arm_joint_controller/command",100);
   ros::Publisher pub_bucket_ctrl = nh.advertise<std_msgs::Float64>("zx135u/backet_joint_controller/command",100);

   // ros::spin();

   ros::Rate loop(10);

   while(ros::ok()){
      pub_rotator_ctrl.publish(rotator_cmd);
      pub_boom_ctrl.publish(boom_cmd);
      pub_arm_ctrl.publish(arm_cmd);
      pub_bucket_ctrl.publish(bucket_cmd);

      // ROS_INFO("ctrl_now: %f, %f, %f, %f", rotator_cmd.data, boom_cmd.data, arm_cmd.data, bucket_cmd.data);

      ros::spinOnce();
      loop.sleep();
   }

   return 0;
}