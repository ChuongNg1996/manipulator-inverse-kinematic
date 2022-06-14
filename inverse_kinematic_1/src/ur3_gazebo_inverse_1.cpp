#include "ros/ros.h" 
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>

//-------------         User Variables          -------------//
static const float joint_tolerance = 0.1;                             // Joint tolerance (rad)
static const int joint_num = 6;                                     // NUmber of joints
static const float home_joint[joint_num] = {2,0,3,0,3,0};           // Home position in joint (rad)

//-------------         Generic Variables       -------------//                              
static bool joint_achieved = 0;                                     // Check if joint angle is reached within joint tolerance
static bool command_en = 0;                                         // Check if there is command for the joint
float error[joint_num];                                             // Stores difference in command and feedback
std_msgs::Float64MultiArray joint_pose;

void JointStateCallback (const sensor_msgs::JointState& msg)
{
    /*
    For ur3 model only, other model needs to be rechecked like this again
    command[0] = joint[2]
    command[1] = joint[1]
    command[2] = joint[0]
    command[3] = joint[3]
    command[4] = joint[4]
    command[5] = joint[5]
    Experiment to find positive direction of z (the revolute joint) on each joint
    */

   if (command_en == 1)
   {
        error[0] = abs(joint_pose.data[0] - msg.position[2]); 
        error[1] = abs(joint_pose.data[1] - msg.position[1]); 
        error[2] = abs(joint_pose.data[2] - msg.position[0]); 
        error[3] = abs(joint_pose.data[3] - msg.position[3]); 
        error[4] = abs(joint_pose.data[4] - msg.position[4]); 
        error[5] = abs(joint_pose.data[5] - msg.position[5]); 

       if ((error[0] < joint_tolerance) && (error[1] < joint_tolerance) 
       && (error[2] < joint_tolerance) && (error[3] < joint_tolerance) 
       && (error[4] < joint_tolerance) && (error[5] < joint_tolerance))

       {
           joint_achieved = 1;
       }
   }
}
 


int main(int argc, char **argv)
{
    //-------------         Initialization      -------------//
    ros::init(argc, argv, "Manual_Control_Controller");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //-------------         Topics              -------------//
    ros::Subscriber joint_state =  n.subscribe("/joint_states", 1, JointStateCallback);
    ros::Publisher joint_command = n.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);


    //-------------         Home Position        -------------//
    for (int i = 0; i < joint_num; i++)
    {
        joint_pose.data.push_back(home_joint[i]);
    }
    command_en = 1;
    while (joint_achieved == 0) joint_command.publish(joint_pose);
    command_en = 0;
    joint_achieved = 0;

    while (ros::ok())
    {
        
    }   
    return 0; 
}


/*
    https://answers.ros.org/question/380890/unable-to-publish-float64multiarray-in-python/ 

    // for (int i = 0; i < joint_pose.data.size(); i++)
    // {
    //     std::cout << joint_pose.data[i] << std::endl;
    // }

    "publishing and latching message for 3.0 seconds" -> Publishing a message takes time
    Should never compare int with float, no result. 
*/