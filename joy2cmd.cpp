#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include <stdint.h>

ros::Publisher cmd_pub;
sensor_msgs::JointState js_cmd;


void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
   //msg->axes[0]->Left_Stick_LR->Arm->js_cmd.effort[2]
//msg->axes[1]->Left_Stick_UpDown->Rotation->js_cmd.name[0]
//msg->axes[3]->Right_Stick_LR->Backet->js_cmd.effort[3]
//msg->axes[4]->Left_Stick_UpDown->Boom->js_cmd.name[1]

   double max_vel = 0.5;

   // sensor_msgs::JointState js_cmd;
   js_cmd.name.resize(8);
   js_cmd.name[0] = "rotator_joint";
   js_cmd.name[1] = "boom_joint";
   js_cmd.name[2] = "arm_joint";
   js_cmd.name[3] = "backet_joint";
   js_cmd.name[4] = "move";
   js_cmd.name[5] = "rotate";
   js_cmd.name[6] = "excavate";
   js_cmd.name[7] = "dump";

   js_cmd.velocity.resize(8);
   js_cmd.velocity[0] = - msg->axes[1] * max_vel;
   js_cmd.velocity[1] = msg->axes[4] * max_vel;
   js_cmd.velocity[2] = - msg->axes[0] * max_vel;
   js_cmd.velocity[3] = msg->axes[3] * max_vel;

   js_cmd.effort.resize(8);
   js_cmd.effort[4]=msg->buttons[0];
   js_cmd.effort[5]=msg->buttons[1];
   js_cmd.effort[6]=msg->buttons[2];
   js_cmd.effort[7]=msg->buttons[3];


   // cmd_pub.publish(js_cmd);
}

int main(int argc, char**argv){
   ros::init(argc,argv,"joy2cmd");
   ros::NodeHandle nh("joy2cmd");

   cmd_pub = nh.advertise<sensor_msgs::JointState>("js_cmd",100);
   ros::Subscriber sub = nh.subscribe("joy",100,joy_callback);

   // ros::spin();

   ros::Rate loop(10);

   //set the initial value
   js_cmd.name.resize(8);
   js_cmd.name[0] = "rotator_joint";
   js_cmd.name[1] = "boom_joint";
   js_cmd.name[2] = "arm_joint";
   js_cmd.name[3] = "backet_joint";
   js_cmd.name[4] = "move";
   js_cmd.name[5] = "rotate";
   js_cmd.name[6] = "excavate";
   js_cmd.name[7] = "dump";

   js_cmd.velocity.resize(8);
   js_cmd.velocity[0] = 0.0;
   js_cmd.velocity[1] = 0.0;
   js_cmd.velocity[2] = 0.0;
   js_cmd.velocity[3] = 0.0;
   js_cmd.velocity[4] = 0.0;
   js_cmd.velocity[5] = 0.0;
   js_cmd.velocity[6] = 0.0;
   js_cmd.velocity[7] = 0.0;

   js_cmd.effort.resize(8);
   js_cmd.effort[0] = 0.0;
   js_cmd.effort[1] = 0.0;
   js_cmd.effort[2] = 0.0;
   js_cmd.effort[3] = 0.0;
   js_cmd.effort[4] = 0.0;
   js_cmd.effort[5] = 0.0;
   js_cmd.effort[6] = 0.0;
   js_cmd.effort[7] = 0.0;

   double tmp[4]; //in this order: move, rotate, excavate, dump
   double diff[4];
   double js_cmd_before[4];
   double tmp_before=1.0;
   int before_index;

   tmp[0]=0;
   tmp[1]=0;
   tmp[2]=0;
   tmp[3]=0;

   for (int i = 0; i < 4; i++)
   {
      js_cmd_before[i] = 0.0;
   }
   

   while(ros::ok())
   {
   //   cmd_handler.send_cmd();
      diff[0]=0;
      diff[1]=0;
      diff[2]=0;
      diff[3]=0;

      for(int i=0; i<4; i++)
      {
         diff[i] = js_cmd.effort[i+4] - js_cmd_before[i];
         
         if(diff[i]==1.0)
         {
            tmp[before_index] = 0;
            tmp[i] = 1.0; 
            before_index = i;                                 
         }
            
         js_cmd.effort[i+4] = tmp[i];

      }

      cmd_pub.publish(js_cmd);

      js_cmd_before[0] = js_cmd.effort[4]; //move
      js_cmd_before[1] = js_cmd.effort[5]; //rotate
      js_cmd_before[2] = js_cmd.effort[6]; //excavate
      js_cmd_before[3] = js_cmd.effort[7]; //dump


      loop.sleep();
      ros::spinOnce();
   }

   return 0;
}