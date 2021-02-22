#include <ros/ros.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>



float joint_position_now[4];
double joint_position_before[4];
double js_cmd_before[4];

sensor_msgs::JointState cmd_msg;

void js_cmd_callback(const sensor_msgs::JointState::ConstPtr &cmd_msg_before)
{

   cmd_msg.name.resize(8);
   cmd_msg.name[0] = cmd_msg_before->name[0];
   cmd_msg.name[1] = cmd_msg_before->name[1];
   cmd_msg.name[2] = cmd_msg_before->name[2];
   cmd_msg.name[3] = cmd_msg_before->name[3];
   cmd_msg.name[4] = cmd_msg_before->name[4]; //move
   cmd_msg.name[5] = cmd_msg_before->name[5]; //rotate
   cmd_msg.name[6] = cmd_msg_before->name[6]; //excavate
   cmd_msg.name[7] = cmd_msg_before->name[7]; //dump

   cmd_msg.effort.resize(8);
   cmd_msg.effort[0] = cmd_msg_before->effort[0];
   cmd_msg.effort[1] = cmd_msg_before->effort[1];
   cmd_msg.effort[2] = cmd_msg_before->effort[2];
   cmd_msg.effort[3] = cmd_msg_before->effort[3];
   cmd_msg.effort[4] = cmd_msg_before->effort[4]; //move
   cmd_msg.effort[5] = cmd_msg_before->effort[5]; //rotate
   cmd_msg.effort[6] = cmd_msg_before->effort[6]; //excavate
   cmd_msg.effort[7] = cmd_msg_before->effort[7]; //dump

}



int main(int argc, char** argv){
    ros::init(argc, argv, "movement_identification");

    ros::NodeHandle n;
    
    ros::Subscriber js_cmd_sub = n.subscribe("/js_cmd_before", 10, js_cmd_callback);

    ros::Rate loop_rate(10);

    js_cmd_before[0] = 0.0;
    js_cmd_before[1] = 0.0;
    js_cmd_before[2] = 0.0;
    js_cmd_before[3] = 0.0;

   //set the initial value of cmd_msg
      cmd_msg.name.resize(8);
      cmd_msg.name[0] = "rotator_joint";
      cmd_msg.name[1] = "boom_joint";
      cmd_msg.name[2] = "arm_joint";
      cmd_msg.name[3] = "backet_joint";
      cmd_msg.name[4] = "move";
      cmd_msg.name[5] = "rotate";
      cmd_msg.name[6] = "excavate";
      cmd_msg.name[7] = "dump";
     

      cmd_msg.effort.resize(8);
      cmd_msg.effort[0] = 0.0;
      cmd_msg.effort[1] = 0.0;
      cmd_msg.effort[2] = 0.0;
      cmd_msg.effort[3] = 0.0;
      cmd_msg.effort[4] = 0.0;
      cmd_msg.effort[5] = 0.0;
      cmd_msg.effort[6] = 0.0;
      cmd_msg.effort[7] = 0.0;


    while (ros::ok())
    {
        // ros::spinOnce();
                

        if(cmd_msg.effort[4] == 1.0)
        {
            ROS_INFO("Moving");
        }
        else if(cmd_msg.effort[5] == 1.0)
        {
            ROS_INFO("Rotating");
        }
        else if(cmd_msg.effort[6] == 1.0)
        {
            ROS_INFO("Excavating");
        }
        else if(cmd_msg.effort[7] == 1.0)
        {
            ROS_INFO("Dumping");
        }
        else
        {
            ROS_INFO("Enter the movement button");
        }    

        js_cmd_before[0] = cmd_msg.effort[4]; //move
        js_cmd_before[1] = cmd_msg.effort[5]; //rotate
        js_cmd_before[2] = cmd_msg.effort[6]; //excavate
        js_cmd_before[3] = cmd_msg.effort[7]; //dump

        ros::spinOnce();
        loop_rate.sleep();

    }

 return 0;

}