#include "ros/ros.h" 
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <math.h>
#include "geometry_msgs/Point.h"
#include <tuple>

//-------------         User Variables          -------------//
static const float joint_tolerance = 0.001;                              // Joint tolerance (rad)
static const int joint_num = 6;                                         // NUmber of joints
static const float home_joint[joint_num] = {0.001,0.001,0.001,-1.57,-1.57,0.001};       // Home position in joint (rad)
static const float joint_direction[joint_num] = {1,-1,-1,-1,-1,-1};     // Direction of angle
static const int iteration_num = 100000;   // Number of iteration to solve the angle for IK

//-------------         Generic Variables       -------------//                              
static bool joint_achieved = 0;                                     // Check if joint angle is reached within joint tolerance
static bool command_en = 0;                                         // Check if there is command for the joint
float error[joint_num];                                             // Stores difference in command and feedback
static float current_joint[joint_num];                              // Stores current joint angle
static float joint_cmd[joint_num];                              // Stores current joint angle
static bool ik_command = 0;
static bool ik_get_joint_done = 0;
static float current_pose[3];
static float x_step;
static float y_step;
static float z_step;
static float joint_step[4];
static float goal_pose[3] = {0.4,0.2,0.2};
std::tuple<float, float, float> fk_return;
std::tuple<float, float, float, float, float, float, float, float, float> pij_return;

std_msgs::Float64MultiArray joint_pose_cmd;

void JointStateCallback (const sensor_msgs::JointState& msg)
{
    /*
    For ur3 model only, other model needs to be rechecked like this again
    command[0] = joint[2] = joint 1 = same direction
    command[1] = joint[1] = joint 2 = opposite direction
    command[2] = joint[0] = joint 3 = opposite direction
    command[3] = joint[3] = joint 4 = opposite direction
    command[4] = joint[4] = joint 5
    command[5] = joint[5] = joint 6
    Experiment to find positive direction of z (the revolute joint) on each joint
    */
    if (ik_command == 1)
    {
        // for (int i = 0; i< joint_num; i++)
        // {
        //     current_joint[i] = msg.position[i];
        // }
        current_joint[0] = msg.position[2]; 
        current_joint[1] = msg.position[1]; 
        current_joint[2] = msg.position[0]; 
        current_joint[3] = msg.position[3]; 
        current_joint[4] = msg.position[4]; 
        current_joint[5] = msg.position[5]; 
        ik_get_joint_done = 1;
    }

   if (command_en == 1)
   {
        // for (int i = 0; i< joint_num; i++)
        // {
        //     error[i] = abs(joint_pose_cmd.data[i] - msg.position[i]); 
        // }
        error[0] = abs(joint_pose_cmd.data[0] - msg.position[2]); 
        error[1] = abs(joint_pose_cmd.data[1] - msg.position[1]); 
        error[2] = abs(joint_pose_cmd.data[2] - msg.position[0]); 
        error[3] = abs(joint_pose_cmd.data[3] - msg.position[3]); 
        error[4] = abs(joint_pose_cmd.data[4] - msg.position[4]); 
        error[5] = abs(joint_pose_cmd.data[5] - msg.position[5]); 

       if ((error[0] < joint_tolerance) && (error[1] < joint_tolerance) 
       && (error[2] < joint_tolerance) && (error[3] < joint_tolerance) 
       && (error[4] < joint_tolerance) && (error[5] < joint_tolerance))

       {
           joint_achieved = 1;
       }
   }
}

void PoseCommandCallback (const geometry_msgs::Point& msg)
{
    goal_pose[0] = msg.x;
    goal_pose[1] = msg.y;
    goal_pose[2] = msg.z;
    ik_command = 1;
}

//-------------         Forward Kinematics      -------------//

std::tuple<float, float, float> forward_kinematic (float theta_1, float theta_2, float theta_3, float theta_4)
{
    float x = ((0.1218*cos(theta_1 - theta_2)) + (0.1066*cos(theta_1 + theta_2 + theta_3)) + (0.1066*cos(theta_2 - theta_1 + theta_3)) + (0.1218*cos(theta_1 + theta_2)));
    float y = ((0.1218*sin(theta_1 - theta_2)) + (0.1066*sin(theta_1 + theta_2 + theta_3)) - (0.1066*sin(theta_2 - theta_1 + theta_3)) + (0.1218*sin(theta_1 + theta_2)));
    float z = ((0.2132*sin(theta_2 + theta_3)) + (0.2435*sin(theta_2)) + 0.1519);   
    return std::make_tuple(x,y,z);
}

//-------------         Inverse Kinematics      -------------//

std::tuple<float, float, float, float, float, float, float, float, float > pseudoinverse_jacobian (float theta_1, float theta_2, float theta_3, float theta_4)
{
    float inv_j_11 = (-(20000*sin(theta_1))/(4264*cos(theta_2 + theta_3) + 4871*cos(theta_2)));
    float inv_j_12 = ((20000*cos(theta_1))/(4264*cos(theta_2 + theta_3) + 4871*cos(theta_2)));
    float inv_j_13 = (-(1.2246e-12)/(4264*cos(theta_2 + theta_3) + 4871*cos(theta_2)));
    float inv_j_21 = (-(10000*cos(theta_1 - theta_3) + 8.7538e+03*cos(2*theta_2 - theta_1 + 2*theta_3) + 10000*cos(theta_1 + 2*theta_2 + theta_3) + 10000*cos(theta_1 + theta_3) + 1.7508e+04*cos(theta_1) + 10000*cos(2*theta_2 - theta_1 + theta_3) + 8.7538e+03*cos(theta_1 + 2*theta_2 + 2*theta_3))/(4871*sin(theta_2 - theta_3) - 4264*sin(theta_2 + 2*theta_3) - 4871*sin(theta_2 + theta_3) + 4264*sin(theta_2)));
    float inv_j_22 = (-(10000*sin(theta_1 - theta_3) - 8.7538e+03*sin(2*theta_2 - theta_1 + 2*theta_3) + 10000*sin(theta_1 + 2*theta_2 + theta_3) + 10000*sin(theta_1 + theta_3) + 1.7508e+04*sin(theta_1) - 10000*sin(2*theta_2 - theta_1 + theta_3) + 8.7538e+03*sin(theta_1 + 2*theta_2 + 2*theta_3))/(4871*sin(theta_2 - theta_3) - 4264*sin(theta_2 + 2*theta_3) - 4871*sin(theta_2 + theta_3) + 4264*sin(theta_2)));
    float inv_j_23 = (-(20000*sin(2*theta_2 + theta_3) + 1.7508e+04*sin(2*theta_2 + 2*theta_3) + 20000*sin(theta_3))/(4871*sin(theta_2 - theta_3) - 4264*sin(theta_2 + 2*theta_3) - 4871*sin(theta_2 + theta_3) + 4264*sin(theta_2)));
    float inv_j_31 = (-(2.3452*cos(theta_1 - theta_2) + 2.0530*cos(theta_1 + theta_2 + theta_3) + 2.0530*cos(theta_2 - theta_1 + theta_3) + 2.3452*cos(theta_1 + theta_2))/(sin(theta_3)));
    float inv_j_32 = (-(2.3452*sin(theta_1 - theta_2) + 2.0530*sin(theta_1 + theta_2 + theta_3) - 2.0530*sin(theta_2 - theta_1 + theta_3) + 2.3452*sin(theta_1 + theta_2))/(sin(theta_3)));
    float inv_j_33 = (-(4.1059*sin(theta_2 + theta_3) + 4.6904*sin(theta_2))/(sin(theta_3)));

    return std::make_tuple(inv_j_11,inv_j_12,inv_j_13,inv_j_21,inv_j_22,inv_j_23,inv_j_31,inv_j_32,inv_j_33);
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
    ros::Subscriber goal_command =  n.subscribe("/pose_command", 1, PoseCommandCallback);
    ros::Publisher joint_command = n.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);


    //-------------         Joint Command: Home Position        -------------//
    std::copy (home_joint,home_joint + joint_num,joint_cmd);
    for (int i = 0; i < joint_num; i++)
    {
        joint_pose_cmd.data.push_back(joint_cmd[i]);
    }
    command_en = 1;
    while (joint_achieved == 0) joint_command.publish(joint_pose_cmd);
    joint_pose_cmd.data.clear();
    command_en = 0;
    joint_achieved = 0;



    //-------------         ROS Loop       -------------//
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if (ik_command == 1)
        {
            while (ik_get_joint_done == 0);
            ik_get_joint_done = 0;
            ik_command = 0;

            //-------------         Joint Direction       -------------//
            current_joint[0] = current_joint[0] * joint_direction[0];
            current_joint[1] = current_joint[1] * joint_direction[1];
            current_joint[2] = current_joint[2] * joint_direction[2];
            current_joint[3] = current_joint[3] * joint_direction[3];

            //-------------         Compute Current Pose       -------------//
            fk_return = forward_kinematic(current_joint[0],current_joint[1],current_joint[2],current_joint[3]);
            current_pose[0] = std::get<0>(fk_return);
            //std::cout << current_pose[0] << std::endl;
            current_pose[1] = std::get<1>(fk_return);
            //std::cout << current_pose[1] << std::endl;
            current_pose[2] = std::get<2>(fk_return);
            //std::cout << current_pose[2] << std::endl;

            //-------------         Get delta of each components    -------------//
            x_step = (goal_pose[0]-current_pose[0])/iteration_num;
            y_step = (goal_pose[1]-current_pose[1])/iteration_num;
            z_step = (goal_pose[2]-current_pose[2])/iteration_num;
            
            for (int i = 0; i < iteration_num; i++) 
            {
                //-------------         Compute (pseudo)inversed Jacobian matrix    -------------//
                pij_return = pseudoinverse_jacobian(current_joint[0],current_joint[1],current_joint[2],current_joint[3]);
                float inv_j_11_r = std::get<0>(pij_return);
                float inv_j_12_r = std::get<1>(pij_return);
                float inv_j_13_r = std::get<2>(pij_return);

                float inv_j_21_r = std::get<3>(pij_return);
                float inv_j_22_r = std::get<4>(pij_return);
                float inv_j_23_r = std::get<5>(pij_return);

                float inv_j_31_r = std::get<6>(pij_return);
                float inv_j_32_r = std::get<7>(pij_return);
                float inv_j_33_r = std::get<8>(pij_return);

                float inv_j_41_r = 0;
                float inv_j_42_r = 0;
                float inv_j_43_r = 0;

                //-------------         Compute command steps of each joint    -------------//
                joint_step[0] = inv_j_11_r*x_step + inv_j_12_r*y_step + inv_j_13_r*z_step;
                joint_step[1] = inv_j_21_r*x_step + inv_j_22_r*y_step + inv_j_23_r*z_step;
                joint_step[2] = inv_j_31_r*x_step + inv_j_32_r*y_step + inv_j_33_r*z_step;
                joint_step[3] = inv_j_41_r*x_step + inv_j_42_r*y_step + inv_j_43_r*z_step;

                //-------------         Compute command joint angles    -------------//
                current_joint[0] = current_joint[0] + joint_step[0];
                current_joint[1] = current_joint[1] + joint_step[1];
                current_joint[2] = current_joint[2] + joint_step[2];
                current_joint[3] = current_joint[3] + joint_step[3];
                current_joint[4] = current_joint[4]; 
                current_joint[5] = current_joint[5];

                // Repeat till complete iteration
            }
            //-------------         Joint Command: To desired pose        -------------//
            current_joint[0] = current_joint[0] * joint_direction[0];
            current_joint[1] = current_joint[1] * joint_direction[1];
            current_joint[2] = current_joint[2] * joint_direction[2];
            //current_joint[3] = current_joint[3] * joint_direction[3];
            //current_joint[3] = current_joint[3] + (current_joint[2] + current_joint[1]);
            current_joint[3] = home_joint[3] - (current_joint[2] + current_joint[1]);

            //-------------         Display Joint Command        -------------//
            std::cout << "Joint 1 Command: " << current_joint[0] << std::endl;
            std::cout << "Joint 2 Command: " << current_joint[1] << std::endl;
            std::cout << "Joint 2 Command: " << current_joint[2] << std::endl;
            std::cout << "Joint 3 Command: " << current_joint[3] << std::endl;

            //-------------------------//
            
            //-------------         Joint Command: To goal        -------------//
            std::copy (current_joint,current_joint + joint_num,joint_cmd);
            for (int i = 0; i < joint_num; i++)
            {
                joint_pose_cmd.data.push_back(joint_cmd[i]);
            }
            command_en = 1;
            while (joint_achieved == 0) joint_command.publish(joint_pose_cmd);
            command_en = 0;
            joint_achieved = 0;
            joint_pose_cmd.data.clear();
            std::cout << "Goal reach." << std::endl;
        }
        loop_rate.sleep();
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