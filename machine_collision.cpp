#include <ros/ros.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>


double base_x, base_y, base_z, base_yaw, rotator_yaw;

double human_x, human_y, human_z;
double machine_x, machine_y, machine_z;
double r_bucket_end, r_bucket, r_arm, r_boom;


float joint_position_now[4];
double joint_position_before[4];

double my_time, my_time2, my_time1;
double my_time_before = 0;

double machine_distance_before = 0;

double machine_lx = 3.0;
double machine_ly = 1.5;
double machine_lz = 1.0;

sensor_msgs::JointState cmd_msg;



void baseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    base_x = msg->pose[5].position.x;
    base_y = msg->pose[5].position.y;
    base_z = msg->pose[5].position.z;
    base_yaw = tf::getYaw(msg->pose[5].orientation);
}


void machineCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    machine_x = msg->pose[2].position.x;
    machine_y = msg->pose[2].position.y;
    machine_z = msg->pose[2].position.z;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    rotator_yaw = msg->position[10];
}

void humanCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    human_x = msg->pose[1].position.x;
    human_y = msg->pose[1].position.y;
    human_z = msg->pose[1].position.z;
}

void timeCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{
   my_time1 = msg->clock.sec;
   my_time2 = msg->clock.nsec;
   my_time = my_time1 + my_time2 * pow(10.0, -9);

}


void joint_states_callback(const sensor_msgs::JointState::ConstPtr& state_msg)
{
   // ROS_INFO("sub state!");

   joint_position_now[0] = state_msg->position[10]; //rotator
   joint_position_now[1] = state_msg->position[3]; //boom
   joint_position_now[2] = state_msg->position[0]; //arm
   joint_position_now[3] = state_msg->position[2]; //bucket

//    ROS_INFO("joint_now: %f, %f, %f, %f", joint_position_now[0], joint_position_now[1], joint_position_now[2], joint_position_now[3]);
}

void js_cmd_callback(const sensor_msgs::JointState::ConstPtr& cmd_msg_before){
   // sensor_msgs::JointState cmd_msg_after;

   cmd_msg.name.resize(8);
   cmd_msg.name[0] = cmd_msg_before->name[0];
   cmd_msg.name[1] = cmd_msg_before->name[1];
   cmd_msg.name[2] = cmd_msg_before->name[2];
   cmd_msg.name[3] = cmd_msg_before->name[3];
   cmd_msg.name[4] = cmd_msg_before->name[4]; //move
   cmd_msg.name[5] = cmd_msg_before->name[5]; //rotate
   cmd_msg.name[6] = cmd_msg_before->name[6]; //excavate
   cmd_msg.name[7] = cmd_msg_before->name[7]; //machine

   cmd_msg.velocity.resize(8);
   cmd_msg.velocity[0]=cmd_msg_before->velocity[0];
   cmd_msg.velocity[1]=cmd_msg_before->velocity[1];
   cmd_msg.velocity[2]=cmd_msg_before->velocity[2];
   cmd_msg.velocity[3]=cmd_msg_before->velocity[3];
   cmd_msg.velocity[4]=cmd_msg_before->velocity[4];
   cmd_msg.velocity[5]=cmd_msg_before->velocity[5];
   cmd_msg.velocity[6]=cmd_msg_before->velocity[6];
   cmd_msg.velocity[7]=cmd_msg_before->velocity[7];

   cmd_msg.effort.resize(8);
   cmd_msg.effort[0] = cmd_msg_before->effort[0];
   cmd_msg.effort[1] = cmd_msg_before->effort[1];
   cmd_msg.effort[2] = cmd_msg_before->effort[2];
   cmd_msg.effort[3] = cmd_msg_before->effort[3];
   cmd_msg.effort[4] = cmd_msg_before->effort[4]; //move
   cmd_msg.effort[5] = cmd_msg_before->effort[5]; //rotate
   cmd_msg.effort[6] = cmd_msg_before->effort[6]; //excavate
   cmd_msg.effort[7] = cmd_msg_before->effort[7]; //machine

}


double vector_magnitude_function(double a[3])
{
    return pow(a[0], 2.0) + pow(a[1], 2.0) + pow(a[2], 2.0);
}

double absolute_value_function(double a)
{
    if (a >= 0)
    {
        return a;
    }
    else
    {
        return -a;
    }
}

double inner_product_function(double a[3], double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double cross_product_function(double a[3], double b[3], double c[3])
{
    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = a[2] * b[0] - b[2] * a[0];
    c[2] = a[0] * b[1] - b[0] * a[1];
}

double calculate_d_min_function(double x0_boom_arm[3], double x0_arm_bucket[3], double x0_bucket_end[3], double machine_vertex[3], double m_boom_arm[3], double m_arm_bucket[3], double m_bucket_end[3], double m_number[3])
{
    double vector_boom_arm1[3], vector_boom_arm21[3], vector_boom_arm22[3];
    double vector_arm_bucket1[3], vector_arm_bucket21[3], vector_arm_bucket22[3];
    double vector_bucket_end1[3], vector_bucket_end21[3], vector_bucket_end22[3];
                

    for (int i = 0; i < 3; i++)
    {
        vector_boom_arm1[i] = x0_boom_arm[i] - machine_vertex[i];
        vector_boom_arm21[i] = vector_magnitude_function(m_number) * m_boom_arm[i] - inner_product_function(m_boom_arm, m_number) * m_number[i];
        vector_boom_arm22[i] = inner_product_function(m_boom_arm, m_number) * m_boom_arm[i] - vector_magnitude_function(m_boom_arm) * m_number[i];
    }    

    for (int i = 0; i < 3; i++)
    {
        vector_arm_bucket1[i] = x0_arm_bucket[i] - machine_vertex[i];
        vector_arm_bucket21[i] = vector_magnitude_function(m_number) * m_arm_bucket[i] - inner_product_function(m_arm_bucket, m_number) * m_number[i];
        vector_arm_bucket22[i] = inner_product_function(m_arm_bucket, m_number) * m_arm_bucket[i] - vector_magnitude_function(m_arm_bucket) * m_number[i];
    }    

    for (int i = 0; i < 3; i++)
    {
        vector_bucket_end1[i] = x0_bucket_end[i] - machine_vertex[i];
        vector_bucket_end21[i] = vector_magnitude_function(m_number) * m_bucket_end[i] - inner_product_function(m_bucket_end, m_number) * m_number[i];
        vector_bucket_end22[i] = inner_product_function(m_bucket_end, m_number) * m_bucket_end[i] - vector_magnitude_function(m_bucket_end) * m_number[i];
    }    


    double t_boom_arm, tt_boom_arm;
    double t_arm_bucket, tt_arm_bucket;
    double t_bucket_end, tt_bucket_end;

    t_boom_arm = inner_product_function(vector_boom_arm1, vector_boom_arm21) / (pow(inner_product_function(m_boom_arm, m_number), 2) - vector_magnitude_function(m_boom_arm) * vector_magnitude_function(m_number));
    tt_boom_arm = inner_product_function(vector_boom_arm1, vector_boom_arm22) / (pow(inner_product_function(m_boom_arm, m_number), 2) - vector_magnitude_function(m_boom_arm) * vector_magnitude_function(m_number));

    t_arm_bucket = inner_product_function(vector_arm_bucket1, vector_arm_bucket21) / (pow(inner_product_function(m_arm_bucket, m_number), 2) - vector_magnitude_function(m_arm_bucket) * vector_magnitude_function(m_number));
    tt_arm_bucket = inner_product_function(vector_arm_bucket1, vector_arm_bucket22) / (pow(inner_product_function(m_arm_bucket, m_number), 2) - vector_magnitude_function(m_arm_bucket) * vector_magnitude_function(m_number));

    t_bucket_end = inner_product_function(vector_bucket_end1, vector_bucket_end21) / (pow(inner_product_function(m_bucket_end, m_number), 2) - vector_magnitude_function(m_bucket_end) * vector_magnitude_function(m_number));
    tt_bucket_end = inner_product_function(vector_bucket_end1, vector_bucket_end22) / (pow(inner_product_function(m_bucket_end, m_number), 2) - vector_magnitude_function(m_bucket_end) * vector_magnitude_function(m_number));
     
    double cross_product_boom_arm[3], cross_product_arm_bucket[3], cross_product_bucket_end[3];
    cross_product_function(m_boom_arm, m_number, cross_product_boom_arm);
    cross_product_function(m_arm_bucket, m_number, cross_product_arm_bucket);
    cross_product_function(m_bucket_end, m_number, cross_product_bucket_end);


    double d_min1_arm_bucket, d_min1_boom_arm, d_min1_bucket_end;

    if ((t_bucket_end >= 0 && t_bucket_end <= 1) && (tt_bucket_end >= 0 && tt_bucket_end <= 1))
    {
        d_min1_bucket_end = absolute_value_function(inner_product_function(vector_bucket_end1, cross_product_bucket_end)) / pow(vector_magnitude_function(cross_product_bucket_end), 0.5);
    }
    else
    {
        double parameter;
        parameter = -inner_product_function(m_bucket_end, vector_bucket_end1) / vector_magnitude_function(m_bucket_end);
        
        double vector_bucket_end11[3];
        for (int i = 0; i < 3; i++)
        {
            vector_bucket_end11[i] = vector_bucket_end1[i] + m_bucket_end[i];
        }

        if (parameter >= 0 && parameter <= 1)
        {
            d_min1_bucket_end = pow((vector_magnitude_function(m_bucket_end) * vector_magnitude_function(vector_bucket_end1) - pow(inner_product_function(m_bucket_end, vector_bucket_end1), 2)) / vector_magnitude_function(m_bucket_end), 0.5);
        }
        else if (parameter < 0)
        {
            d_min1_bucket_end = pow(vector_magnitude_function(vector_bucket_end1), 0.5);
        }        
        else if (parameter > 1)
        {
            d_min1_bucket_end = pow(vector_magnitude_function(vector_bucket_end11), 0.5);
        }        
    }


    if ((t_arm_bucket >= 0 && t_arm_bucket <= 1) && (tt_arm_bucket >= 0 && tt_arm_bucket <= 1))
    {
        d_min1_arm_bucket = absolute_value_function(inner_product_function(vector_arm_bucket1, cross_product_arm_bucket)) / pow(vector_magnitude_function(cross_product_arm_bucket), 0.5);
    }
    else
    {
        double parameter;
        parameter = -inner_product_function(m_arm_bucket, vector_arm_bucket1) / vector_magnitude_function(m_arm_bucket);
        
        double vector_arm_bucket11[3];
        for (int i = 0; i < 3; i++)
        {
            vector_arm_bucket11[i] = vector_arm_bucket1[i] + m_arm_bucket[i];
        }

        if (parameter >= 0 && parameter <= 1)
        {
            d_min1_arm_bucket = pow((vector_magnitude_function(m_arm_bucket) * vector_magnitude_function(vector_arm_bucket1) - pow(inner_product_function(m_arm_bucket, vector_arm_bucket1), 2)) / vector_magnitude_function(m_arm_bucket), 0.5);
        }
        else if (parameter < 0)
        {
            d_min1_arm_bucket = pow(vector_magnitude_function(vector_arm_bucket1), 0.5);
        }        
        else if (parameter > 1)
        {
            d_min1_arm_bucket = pow(vector_magnitude_function(vector_arm_bucket11), 0.5);
        }        
    }

    if ((t_boom_arm >= 0 && t_boom_arm <= 1) && (tt_boom_arm >= 0 && tt_boom_arm <= 1))
    {
        d_min1_boom_arm = absolute_value_function(inner_product_function(vector_boom_arm1, cross_product_boom_arm)) / pow(vector_magnitude_function(cross_product_boom_arm), 0.5);
    }
    else
    {
        double parameter;
        parameter = -inner_product_function(m_boom_arm, vector_boom_arm1) / vector_magnitude_function(m_boom_arm);
        
        double vector_boom_arm11[3];
        for (int i = 0; i < 3; i++)
        {
            vector_boom_arm11[i] = vector_boom_arm1[i] + m_boom_arm[i];
        }

        if (parameter >= 0 && parameter <= 1)
        {
            d_min1_boom_arm = pow((vector_magnitude_function(m_boom_arm) * vector_magnitude_function(vector_boom_arm1) - pow(inner_product_function(m_boom_arm, vector_boom_arm1), 2)) / vector_magnitude_function(m_boom_arm), 0.5);
        }
        else if (parameter < 0)
        {
            d_min1_boom_arm = pow(vector_magnitude_function(vector_boom_arm1), 0.5);
        }        
        else if (parameter > 1)
        {
            d_min1_boom_arm = pow(vector_magnitude_function(vector_boom_arm11), 0.5);
        }
    }

    // double d_min1;

    if (d_min1_boom_arm < d_min1_arm_bucket)
    {
        if (d_min1_boom_arm < d_min1_bucket_end)
        {
            return d_min1_boom_arm;
        }
        else 
        {
            return d_min1_bucket_end;
        }     
    }
    else
    {
        if (d_min1_arm_bucket < d_min1_bucket_end)
        {
            return d_min1_arm_bucket;
        }
        else
        {
            return d_min1_bucket_end;
        }
        
        
    }
    
    

}

int main(int argc, char** argv){
    ros::init(argc, argv, "machine_collision");

    ros::NodeHandle n;
    ros::Subscriber base_sub = n.subscribe("/gazebo/model_states", 100, baseCallback);    
    ros::Subscriber joint_sub = n.subscribe("zx135u/joint_states", 100, jointCallback);
    ros::Subscriber human1_sub = n.subscribe("/gazebo/model_states", 100, humanCallback);
    ros::Subscriber machine_sub = n.subscribe("/gazebo/model_states", 100, machineCallback);
    
    ros::Subscriber sub_cmd = n.subscribe("js_cmd_after", 100, js_cmd_callback);


    ros::Subscriber time_sub = n.subscribe("/clock", 100, timeCallback);

    ros::Subscriber sub_state = n.subscribe("zx135u/joint_states", 100, joint_states_callback);


    tf::TransformListener listener;

    ros::Rate loop_rate(10);

    double bucket_end_x, bucket_end_y, bucket_end_z;
    double bucket_x, bucket_y, bucket_z;
    double arm_x, arm_y, arm_z;
    double boom_x, boom_y, boom_z;

    double true_bucket_end_x, true_bucket_end_y, true_bucket_end_z;
    double true_bucket_x, true_bucket_y, true_bucket_z;
    double true_arm_x, true_arm_y, true_arm_z;
    double true_boom_x, true_boom_y, true_boom_z;

    joint_position_before[0] = 0;
    joint_position_before[1] = 0;
    joint_position_before[2] = 0;
    joint_position_before[3] = 0;

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

    cmd_msg.velocity.resize(8);
    for (int i = 0; i < 8; i++)
    {
        cmd_msg.velocity[i] = 0.0;        
    }
    
    cmd_msg.effort.resize(8);            
    for (int i = 0; i < 8; i++)
    {
        cmd_msg.effort[i] = 0.0;
    }

    int num_collision = 0;
    int num_warning = 0;




    while (ros::ok())
    {
        // ros::spinOnce();
        tf::StampedTransform transform_bucket_end, transform_bucket, transform_arm, transform_boom;

        try
        {   
            listener.lookupTransform("/base_link", "/backet_end_link", ros::Time(0), transform_bucket_end);
            bucket_end_x = transform_bucket_end.getOrigin().x();
            bucket_end_y = transform_bucket_end.getOrigin().y();
            bucket_end_z = transform_bucket_end.getOrigin().z(); 


            listener.lookupTransform("/base_link", "/backet_link", ros::Time(0), transform_bucket);
            bucket_x = transform_bucket.getOrigin().x();
            bucket_y = transform_bucket.getOrigin().y();
            bucket_z = transform_bucket.getOrigin().z(); 

            listener.lookupTransform("/base_link", "/arm_link", ros::Time(0), transform_arm);
            arm_x = transform_arm.getOrigin().x();
            arm_y = transform_arm.getOrigin().y();
            arm_z = transform_arm.getOrigin().z();

            listener.lookupTransform("/base_link", "/boom_link", ros::Time(0), transform_boom);
            boom_x = transform_boom.getOrigin().x();
            boom_y = transform_boom.getOrigin().y();
            boom_z = transform_boom.getOrigin().z();


        }
        catch(tf::TransformException &ex)
        {
           ROS_ERROR("%s", ex.what());
           ros::Duration(1.0).sleep();
           continue;
        }

        true_bucket_end_x = base_x + r_bucket_end * cos(base_yaw + rotator_yaw);
        true_bucket_end_y = base_y + r_bucket_end * sin(base_yaw + rotator_yaw);
        true_bucket_end_z = base_z + bucket_end_z;
        r_bucket_end = pow(pow(bucket_end_x, 2.0) + pow(bucket_end_y, 2.0), 0.5); 

        true_bucket_x = base_x + r_bucket * cos(base_yaw + rotator_yaw);
        true_bucket_y = base_y + r_bucket * sin(base_yaw + rotator_yaw);
        true_bucket_z = base_z + bucket_z;
        r_bucket = pow(pow(bucket_x, 2.0) + pow(bucket_y, 2.0), 0.5); 

        true_arm_x = base_x + r_arm * cos(base_yaw + rotator_yaw);
        true_arm_y = base_y + r_arm * sin(base_yaw + rotator_yaw);
        true_arm_z = base_z + arm_z;
        r_arm = pow(pow(arm_x, 2.0) + pow(arm_y, 2.0), 0.5);

        true_boom_x = base_x + r_boom * cos(base_yaw + rotator_yaw);
        true_boom_y = base_y + r_boom * sin(base_yaw + rotator_yaw);
        true_boom_z = base_z + boom_z;
        r_boom = pow(pow(boom_x, 2.0) + pow(boom_y, 2.0), 0.5);


      double machine_vertex_0[3], machine_vertex_1[3], machine_vertex_2[3], machine_vertex_3[3], machine_vertex_4[3], machine_vertex_5[3], machine_vertex_6[3], machine_vertex_7[3];
      double machine_vertex_8[3], machine_vertex_9[3], machine_vertex_10[3], machine_vertex_11[3], machine_vertex_12[3], machine_vertex_13[3], machine_vertex_14[3], machine_vertex_15[3];
      double machine_vertex_16[3], machine_vertex_17[3], machine_vertex_18[3], machine_vertex_19[3];

      machine_vertex_0[0] = machine_x - machine_lx / 2;
      machine_vertex_0[1] = machine_y - machine_ly / 2;
      machine_vertex_0[2] = machine_z - machine_lz / 2;

      machine_vertex_1[0] = machine_x + machine_lx / 2;
      machine_vertex_1[1] = machine_y - machine_ly / 2;
      machine_vertex_1[2] = machine_z - machine_lz / 2;

      machine_vertex_2[0] = machine_x + machine_lx / 2;
      machine_vertex_2[1] = machine_y - machine_ly / 2;
      machine_vertex_2[2] = machine_z + machine_lz / 2;

      machine_vertex_3[0] = machine_x - machine_lx / 2;
      machine_vertex_3[1] = machine_y - machine_ly / 2;
      machine_vertex_3[2] = machine_z + machine_lz / 2;

      machine_vertex_4[0] = machine_x - machine_lx / 2;
      machine_vertex_4[1] = machine_y + machine_ly / 2;
      machine_vertex_4[2] = machine_z - machine_lz / 2;

      machine_vertex_5[0] = machine_x + machine_lx / 2;
      machine_vertex_5[1] = machine_y + machine_ly / 2;
      machine_vertex_5[2] = machine_z - machine_lz / 2;

      machine_vertex_6[0] = machine_x + machine_lx / 2;
      machine_vertex_6[1] = machine_y + machine_ly / 2;
      machine_vertex_6[2] = machine_z + machine_lz / 2;

      machine_vertex_7[0] = machine_x - machine_lx / 2;
      machine_vertex_7[1] = machine_y + machine_ly / 2;
      machine_vertex_7[2] = machine_z + machine_lz / 2;

      machine_vertex_8[0] = machine_x;
      machine_vertex_8[1] = machine_y - machine_ly / 2;
      machine_vertex_8[2] = machine_z - machine_lz / 2;

      machine_vertex_9[0] = machine_x + machine_lx / 2;
      machine_vertex_9[1] = machine_y;
      machine_vertex_9[2] = machine_z - machine_lz / 2;

      machine_vertex_10[0] = machine_x;
      machine_vertex_10[1] = machine_y + machine_ly / 2;
      machine_vertex_10[2] = machine_z - machine_lz / 2;

      machine_vertex_11[0] = machine_x - machine_lx / 2;
      machine_vertex_11[1] = machine_y;
      machine_vertex_11[2] = machine_z - machine_lz / 2;

      machine_vertex_12[0] = machine_x - machine_lx / 2;
      machine_vertex_12[1] = machine_y - machine_ly / 2;
      machine_vertex_12[2] = machine_z;

      machine_vertex_13[0] = machine_x + machine_lx / 2;
      machine_vertex_13[1] = machine_y - machine_ly / 2;
      machine_vertex_13[2] = machine_z;

      machine_vertex_14[0] = machine_x + machine_lx / 2;
      machine_vertex_14[1] = machine_y + machine_ly / 2;
      machine_vertex_14[2] = machine_z;

      machine_vertex_15[0] = machine_x - machine_lx / 2;
      machine_vertex_15[1] = machine_y + machine_ly / 2;
      machine_vertex_15[2] = machine_z;

      machine_vertex_16[0] = machine_x;
      machine_vertex_16[1] = machine_y - machine_ly / 2;
      machine_vertex_16[2] = machine_z + machine_lz / 2;

      machine_vertex_17[0] = machine_x + machine_lx / 2;
      machine_vertex_17[1] = machine_y;
      machine_vertex_17[2] = machine_z + machine_lz / 2;

      machine_vertex_18[0] = machine_x;
      machine_vertex_18[1] = machine_y + machine_ly / 2;
      machine_vertex_18[2] = machine_z + machine_lz / 2;

      machine_vertex_19[0] = machine_x - machine_lx / 2;
      machine_vertex_19[1] = machine_y;
      machine_vertex_19[2] = machine_z + machine_lz / 2;

      double n_0[3], n_1[3], n_2[3], n_3[3], n_4[3], n_5[3], n_6[3], n_7[3], n_8[3], n_9[3], n_10[3], n_11[3];
      double n_12[3], n_13[3], n_14[3], n_15[3], n_16[3], n_17[3], n_18[3], n_19[3], n_20[3], n_21[3];
        
      n_0[0] = machine_vertex_1[0] - machine_vertex_0[0];
      n_0[1] = machine_vertex_1[1] - machine_vertex_0[1];
      n_0[2] = machine_vertex_1[2] - machine_vertex_0[2];
    
      n_1[0] = machine_vertex_2[0] - machine_vertex_1[0];
      n_1[1] = machine_vertex_2[1] - machine_vertex_1[1];
      n_1[2] = machine_vertex_2[2] - machine_vertex_1[2];

      n_2[0] = machine_vertex_3[0] - machine_vertex_2[0];
      n_2[1] = machine_vertex_3[1] - machine_vertex_2[1];
      n_2[2] = machine_vertex_3[2] - machine_vertex_2[2];

      n_3[0] = machine_vertex_0[0] - machine_vertex_3[0];
      n_3[1] = machine_vertex_0[1] - machine_vertex_3[1];
      n_3[2] = machine_vertex_0[2] - machine_vertex_3[2];
    
      n_4[0] = machine_vertex_5[0] - machine_vertex_4[0];
      n_4[1] = machine_vertex_5[1] - machine_vertex_4[1];
      n_4[2] = machine_vertex_5[2] - machine_vertex_4[2];

      n_5[0] = machine_vertex_6[0] - machine_vertex_5[0];
      n_5[1] = machine_vertex_6[1] - machine_vertex_5[1];
      n_5[2] = machine_vertex_6[2] - machine_vertex_5[2];

      n_6[0] = machine_vertex_7[0] - machine_vertex_6[0];
      n_6[1] = machine_vertex_7[1] - machine_vertex_6[1];
      n_6[2] = machine_vertex_7[2] - machine_vertex_6[2];

      n_7[0] = machine_vertex_4[0] - machine_vertex_7[0];
      n_7[1] = machine_vertex_4[1] - machine_vertex_7[1];
      n_7[2] = machine_vertex_4[2] - machine_vertex_7[2];
    
      n_8[0] = machine_vertex_4[0] - machine_vertex_0[0];
      n_8[1] = machine_vertex_4[1] - machine_vertex_0[1];
      n_8[2] = machine_vertex_4[2] - machine_vertex_0[2];

      n_9[0] = machine_vertex_5[0] - machine_vertex_1[0];
      n_9[1] = machine_vertex_5[1] - machine_vertex_1[1];
      n_9[2] = machine_vertex_5[2] - machine_vertex_1[2];

      n_10[0] = machine_vertex_6[0] - machine_vertex_2[0];
      n_10[1] = machine_vertex_6[1] - machine_vertex_2[1];
      n_10[2] = machine_vertex_6[2] - machine_vertex_2[2];

      n_11[0] = machine_vertex_7[0] - machine_vertex_3[0];
      n_11[1] = machine_vertex_7[1] - machine_vertex_3[1];
      n_11[2] = machine_vertex_7[2] - machine_vertex_3[2];

      n_12[0] = machine_vertex_13[0] - machine_vertex_12[0];
      n_12[1] = machine_vertex_13[1] - machine_vertex_12[1];
      n_12[2] = machine_vertex_13[2] - machine_vertex_12[2];
      
      n_13[0] = machine_vertex_16[0] - machine_vertex_8[0];
      n_13[1] = machine_vertex_16[1] - machine_vertex_8[1];
      n_13[2] = machine_vertex_16[2] - machine_vertex_8[2];

      n_14[0] = machine_vertex_14[0] - machine_vertex_13[0];
      n_14[1] = machine_vertex_14[1] - machine_vertex_13[1];
      n_14[2] = machine_vertex_14[2] - machine_vertex_13[2];

      n_15[0] = machine_vertex_17[0] - machine_vertex_9[0];
      n_15[1] = machine_vertex_17[1] - machine_vertex_9[1];
      n_15[2] = machine_vertex_17[2] - machine_vertex_9[2];

      n_16[0] = machine_vertex_15[0] - machine_vertex_14[0];
      n_16[1] = machine_vertex_15[1] - machine_vertex_14[1];
      n_16[2] = machine_vertex_15[2] - machine_vertex_14[2];

      n_17[0] = machine_vertex_18[0] - machine_vertex_10[0];
      n_17[1] = machine_vertex_18[1] - machine_vertex_10[1];
      n_17[2] = machine_vertex_18[2] - machine_vertex_10[2];

      n_18[0] = machine_vertex_15[0] - machine_vertex_12[0];
      n_18[1] = machine_vertex_15[1] - machine_vertex_12[1];
      n_18[2] = machine_vertex_15[2] - machine_vertex_12[2];

      n_19[0] = machine_vertex_19[0] - machine_vertex_11[0];
      n_19[1] = machine_vertex_19[1] - machine_vertex_11[1];
      n_19[2] = machine_vertex_19[2] - machine_vertex_11[2];

      n_20[0] = machine_vertex_18[0] - machine_vertex_16[0];
      n_20[1] = machine_vertex_18[1] - machine_vertex_16[1];
      n_20[2] = machine_vertex_18[2] - machine_vertex_16[2];

      n_21[0] = machine_vertex_19[0] - machine_vertex_17[0];
      n_21[1] = machine_vertex_19[1] - machine_vertex_17[1];
      n_21[2] = machine_vertex_19[2] - machine_vertex_17[2];

      //define the line between boom and arm
      double x0_boom_arm[3], x0_arm_bucket[3], x0_bucket_end[3];  //define the constant point
      double m_boom_arm[3], m_arm_bucket[3], m_bucket_end[3];  //define the gradient of the line

      x0_boom_arm[0] = true_boom_x;
      x0_boom_arm[1] = true_boom_y;
      x0_boom_arm[2] = true_boom_z;

      x0_arm_bucket[0] = true_arm_x;
      x0_arm_bucket[1] = true_arm_y;
      x0_arm_bucket[2] = true_arm_z;

      x0_bucket_end[0] = true_bucket_x;
      x0_bucket_end[1] = true_bucket_y;
      x0_bucket_end[2] = true_bucket_z;

      m_boom_arm[0] = true_arm_x - true_boom_x;
      m_boom_arm[1] = true_arm_y - true_boom_y;
      m_boom_arm[2] = true_arm_z - true_boom_z;  

      m_arm_bucket[0] = true_bucket_x - true_arm_x;
      m_arm_bucket[1] = true_bucket_y - true_arm_y;
      m_arm_bucket[2] = true_bucket_z - true_arm_z;    

      m_bucket_end[0] = true_bucket_end_x - true_bucket_x;
      m_bucket_end[1] = true_bucket_end_y - true_bucket_y;
      m_bucket_end[2] = true_bucket_end_z - true_bucket_z;  

      double d_min1[22];

      d_min1[0] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_0, m_boom_arm, m_arm_bucket, m_bucket_end, n_0);
      d_min1[1] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_1, m_boom_arm, m_arm_bucket, m_bucket_end, n_1);
      d_min1[2] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_2, m_boom_arm, m_arm_bucket, m_bucket_end, n_2);
      d_min1[3] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_3, m_boom_arm, m_arm_bucket, m_bucket_end, n_3);
      d_min1[4] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_4, m_boom_arm, m_arm_bucket, m_bucket_end, n_4);
      d_min1[5] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_5, m_boom_arm, m_arm_bucket, m_bucket_end, n_5);
      d_min1[6] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_6, m_boom_arm, m_arm_bucket, m_bucket_end, n_6);
      d_min1[7] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_7, m_boom_arm, m_arm_bucket, m_bucket_end, n_7);
      d_min1[8] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_0, m_boom_arm, m_arm_bucket, m_bucket_end, n_8);
      d_min1[9] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_1, m_boom_arm, m_arm_bucket, m_bucket_end, n_9);
      d_min1[10] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_2, m_boom_arm, m_arm_bucket, m_bucket_end, n_10);
      d_min1[11] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_3, m_boom_arm, m_arm_bucket, m_bucket_end, n_11);
      d_min1[12] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_12, m_boom_arm, m_arm_bucket, m_bucket_end, n_12);
      d_min1[13] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_8, m_boom_arm, m_arm_bucket, m_bucket_end, n_13);
      d_min1[14] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_13, m_boom_arm, m_arm_bucket, m_bucket_end, n_14);
      d_min1[15] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_9, m_boom_arm, m_arm_bucket, m_bucket_end, n_15);
      d_min1[16] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_14, m_boom_arm, m_arm_bucket, m_bucket_end, n_16);
      d_min1[17] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_10, m_boom_arm, m_arm_bucket, m_bucket_end, n_17);
      d_min1[18] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_12, m_boom_arm, m_arm_bucket, m_bucket_end, n_18);
      d_min1[19] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_11, m_boom_arm, m_arm_bucket, m_bucket_end, n_19);
      d_min1[20] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_16, m_boom_arm, m_arm_bucket, m_bucket_end, n_20);
      d_min1[21] = calculate_d_min_function(x0_boom_arm, x0_arm_bucket, x0_bucket_end, machine_vertex_17, m_boom_arm, m_arm_bucket, m_bucket_end, n_21);

      double d_shortest_machine;
      int d_shortest_machine_index;

      d_shortest_machine = d_min1[0];
      d_shortest_machine_index = 0;

      for (int i = 0; i < 22; i++)
      {
         if (d_shortest_machine > d_min1[i])
         {
            d_shortest_machine = d_min1[i];
            d_shortest_machine_index = i;
         }
      }


        double threshold_machine = 1.0;

        if ((machine_distance_before > 0.5) && (d_shortest_machine < 0.5) && (cmd_msg.velocity[0]!=0 || cmd_msg.velocity[1]!=0 || cmd_msg.velocity[2]!=0 || cmd_msg.velocity[3]!=0))
        {
            num_collision = num_collision + 1;
        }
        else if ((machine_distance_before > threshold_machine) && (d_shortest_machine < threshold_machine) && (cmd_msg.velocity[0]!=0 || cmd_msg.velocity[1]!=0 || cmd_msg.velocity[2]!=0 || cmd_msg.velocity[3]!=0))
        {
            num_warning = num_warning + 1;
        }
        // ROS_INFO("d is %f\n", d_shortest_machine);
        ROS_INFO("Machine Collision and Warning number is %d, %d\n", num_collision, num_warning);

        // else 
        // {
        //     ROS_INFO("Machine is safe\n");
        // }



        // ROS_INFO("cmd_msg.velocity[0] is %f\n", cmd_msg.velocity[0]);
       

        //reserve the previoud data
        my_time_before = my_time;
        machine_distance_before = d_shortest_machine;

        joint_position_before[0] = joint_position_now[0];
        joint_position_before[1] = joint_position_now[1];
        joint_position_before[2] = joint_position_now[2];
        joint_position_before[3] = joint_position_now[3];


        ros::spinOnce();
        loop_rate.sleep();
    }

 return 0;

}