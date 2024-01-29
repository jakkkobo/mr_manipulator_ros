/*
* This code enables the communication between the real arm robot and the simulated one
* Receives the joint states of the real robot (UART) and publishes them to the simulated robot
* Receives the waypoint pose from the simulated robot and sends it to the real robot (UART)
*/

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <mr_joint_controller/JointTarget.h>
#include <geometry_msgs/Vector3.h>


#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include "serial/serial.h"

extern "C" {
    #include <mr_communication/mts_serial.h>
}

ros::Publisher joint_states_pub;
ros::Subscriber waypoint_sub;
ros::Subscriber set_velocity_sub;
ros::Subscriber velocity_sub;
ros::Publisher position_pub;

//global variables
double joints[3] = {0,0,0};
double read_pose[3] = {0,0,0};
// int flag_receive = 0;
uint16_t read_joints[3] = {0,0,0};

Serial_t serial_t;   //serial port
int flag_write = 0;  //flag to write to serial port

uint16_t theta3_offset_joint = 156;
uint16_t theta2_start = 60;
uint16_t theta2_offset = 15;
uint16_t theta3_start = 60;
uint16_t joint_controller_offset = 90;



void convert_joints(double* read_joints)
{
    //print the joint states
    // printf("joint1: %f\n", read_joints[0]);
    // printf("joint2: %f\n", read_joints[1]);
    // printf("joint3: %f\n", read_joints[2]);
    
    //convert the joint statesto the range -pi to pi
    //joint 1 range [999, 150] -> [-45, 45]
    double joint1 = (450.0 - read_joints[0]) * 90.0 / 849.0;
    //convert to radians [-pi, pi]
    joint1 = joint1 * M_PI / 180.0;

    //joint 2 range [150, 1000] -> [60, -15]
    double joint2 = (820.0 - read_joints[1]) * 75.0 / 340.0;
    joint2 = joint2 - joint_controller_offset + theta2_offset*2; //offset from convertion + normal offset + offset from joint controller
    //invert rotation direction
    joint2 = -joint2 + 360.0;
    //convert to radians [-pi, pi]
    joint2 = joint2 * M_PI / 180.0;
    if (joint2 > M_PI)
        joint2 = joint2 - 2*M_PI;

    //joint 3 range [480, 680] -> [60, 0]
    double joint3 = (680.0 - read_joints[2]) * 60.0 / 200.0;
    // joint3 = joint3 - theta3_start + theta2_offset;
    //convert to radians [-pi, pi]
    joint3 = joint3 * M_PI / 180.0;    

    //save converted joints in global variable
    joints[0] = joint1;
    joints[1] = joint2;
    joints[2] = joint3;

    //print the converted joints
    // printf("joint1: %f\n", joint1);
    // printf("joint2: %f\n", joint2);
    // printf("joint3: %f\n", joint3);
}


void publish_joints()
{
    // uint16_t read_joint[3] = {0,0,0};

    //convert joints to double
    double read_joint_double[3] = {0,0,0};

    read_joint_double[0] = (double)read_joints[0];
    read_joint_double[1] = (double)read_joints[1];
    read_joint_double[2] = (double)read_joints[2];
    //convert the joint states to the range -pi to pi 
    convert_joints(read_joint_double);

    //publish the joint states
    mr_joint_controller::JointTarget joint_target;
    joint_target.header.stamp = ros::Time::now();
    joint_target.joint1 = joints[0];
    joint_target.joint2 = joints[1];
    joint_target.joint3 = joints[2];
    joint_states_pub.publish(joint_target);
        

    
}

void publish_poses()
{
    //publish the joint states
    geometry_msgs::PointStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.point.x = read_pose[0];
    pose.point.y = read_pose[1];
    pose.point.z = read_pose[2];
    position_pub.publish(pose);
}

/*
* Function to convert the joint states to a string
* The string is in the format: j100j200j300, 100,200,300 are the joints angel
* Each joint angle has a convertion to -pi to pi
*/ 
void read()
{
    //read serial port and check for joint states
    char buffer[27];
    // int n = sReadDev(&serial_t, buffer, sizeof(buffer));
    if (flag_write == 0)
    {
        int i=0;
        int nn = 0;
        while (1)
        {
            char helper;
            int n = sReadDev(&serial_t, &helper, sizeof(helper));
            if(helper == '\n')
                break;
            //pass char to buffer
            buffer[i] = helper;
            nn += 1;
            i += 1;
        }
        //print received buffer
        // printf("buffer: %s\n", buffer);
        // printf("buffer size: %d\n", nn);

        // //check if the buffer is empty
        // if(n <= 0)
        // {   
        //     printf("buffer empty\n");
        //     return;
        // }

        //check if the buffer has the correct size
        if(nn != 27)
        {
            printf("buffer size error\n");
            return;
        }

        //check if the buffer has the correct format (j1100 or j2123 or j3123)
        if(buffer[0] != 'j' && buffer[1] != '1' && buffer[15] != 'p' )
        {
            printf("buffer format error\n");
            return;
        }

        

        if(buffer[0] == 'j')
        {
            //convert the buffer to a string
            std::string buffer_string(buffer);

            //get the joint states from the buffer
            std::string joint1_string = buffer_string.substr(2, 3);
            std::string joint2_string = buffer_string.substr(7, 3);
            std::string joint3_string = buffer_string.substr(12, 3);

            //print the joint states
            // printf("joint1: %s\n", joint1_string.c_str());
            // printf("joint2: %s\n", joint2_string.c_str());
            // printf("joint3: %s\n", joint3_string.c_str());

            //convert the joint states to uint16_t
            read_joints[0] = std::stoi(joint1_string);
            read_joints[1] = std::stoi(joint2_string);
            read_joints[2] = std::stoi(joint3_string);

            //print the joint states
            // printf("joint1: %d\n", read_joints[0]);
            // printf("joint2: %d\n", read_joints[1]);
            // printf("joint3: %d\n", read_joints[2]);
            publish_joints();
            // flag_receive = 1;
        }
        if (buffer[15] == 'p')
        {
            //convert the buffer to a string
            std::string buffer_string(buffer);

            //get the pose states from the buffer
            std::string pose1_string = buffer_string.substr(17, 2);
            std::string pose2_string = buffer_string.substr(21, 2);
            std::string pose3_string = buffer_string.substr(24, 2);

            //get y signal in pose [20]
            std::string signal_y = buffer_string.substr(20, 1);


            //print the pose states
            // printf("pose1: %s\n", pose1_string.c_str());
            // printf("pose2: %s\n", pose2_string.c_str());
            // printf("pose3: %s\n", pose3_string.c_str());

            //convert the poses from string to doubles
            read_pose[0] = std::stod(pose1_string);
            read_pose[1] = std::stod(pose2_string);
            read_pose[2] = std::stod(pose3_string);

            //check if y signal is negative
            if(signal_y == "-")
            {
                read_pose[1] = - read_pose[1];
            }

            //convert poses from cm to m
            read_pose[0] = read_pose[0] / 100;
            read_pose[1] = read_pose[1] / 100;
            read_pose[2] = read_pose[2] / 100;
            publish_poses();   
        }
    }   
}


void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%f]", msg->point.x);
    //ROS_INFO("I heard: [%f]", msg->point.y);
    //ROS_INFO("I heard: [%f]", msg->point.z);

    //convert the x, y, z centimeters to meters int
    int x = (int)(msg->point.x * 100);
    int y = (int)(msg->point.y * 100);
    int z = (int)(msg->point.z * 100);

    //get individual digits of the x, y, z
    int x1 = x / 10;
    int x2 = x % 10;
    int y1 = abs(y) / 10;
    int y2 = abs(y) % 10;
    int z1 = z / 10;
    int z2 = z % 10;

    std::string signal;

    //create y signal
    if(y < 0)
        signal = "-";
    else
        signal = "+";

    //Convert the waypoint pose to a string - x10y-10z10
    std::string waypoint_pose = "x" + std::to_string(x1) + std::to_string(x2) + "y" + signal.c_str() + std::to_string(y1) + std::to_string(y2) + "z" + std::to_string(z1) + std::to_string(z2);
    printf("waypoint_pose: %s\n", waypoint_pose.c_str());

    //Convert the string to a char array
    char waypoint_pose_char[100];
    strcpy(waypoint_pose_char, waypoint_pose.c_str());

    //Write the waypoint pose to the serial port
    flag_write = 1;
    sWriteDev(&serial_t, waypoint_pose_char, sizeof(waypoint_pose_char));
    flag_write = 0;
}

void set_velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    //Save the velocities to array
    double set_velocities[3] = {0,0,0};
    set_velocities[0] = msg->x;
    set_velocities[1] = msg->y;
    set_velocities[2] = msg->z;
    //convert velocities from m/s to cm/s
    set_velocities[0] = set_velocities[0] * 100;
    set_velocities[1] = set_velocities[1] * 100;
    set_velocities[2] = set_velocities[2] * 100;
    ////get individual digits of the x, y, z
    int x1 = (int)set_velocities[0] / 10;
    int x2 = (int)set_velocities[0] % 10;
    int y1 = (int)set_velocities[1] / 10;
    int y2 = (int)set_velocities[1] % 10;
    int z1 = (int)set_velocities[2] / 10;
    int z2 = (int)set_velocities[2] % 10;
    //Convert the velocities to a string - svx10y20z30
    std::string set_velocities_string = "svx" + std::to_string(x1) + std::to_string(x2) + "y" + std::to_string(y1) + std::to_string(y2) + "z" + std::to_string(z1) + std::to_string(z2);
    printf("set_velocities: %s\n", set_velocities_string.c_str());
    //Write the velocities to the serial port
    char set_velocities_char[100];
    strcpy(set_velocities_char, set_velocities_string.c_str());
    flag_write = 1;
    sWriteDev(&serial_t, set_velocities_char, sizeof(set_velocities_char));
    flag_write = 0;
}

void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    //Save the velocities to array
    double velocities[3] = {0,0,0};
    velocities[0] = msg->x;
    velocities[1] = msg->y;
    velocities[2] = msg->z;
    //convert velocities from m/s to cm/s
    velocities[0] = velocities[0] * 100;
    velocities[1] = velocities[1] * 100;
    velocities[2] = velocities[2] * 100;
    //get individual digits of the x, y, z
    int x1 = (int)velocities[0] / 10;
    int x2 = (int)velocities[0] % 10;
    int y1 = (int)velocities[1] / 10;
    int y2 = (int)velocities[1] % 10;
    int z1 = (int)velocities[2] / 10;
    int z2 = (int)velocities[2] % 10;
    //create x, y, z signals
    std::string signal_x;
    std::string signal_y;
    std::string signal_z;
    if(velocities[0] < 0)
        signal_x = "-";
    else
        signal_x = "+";
    if(velocities[1] < 0)
        signal_y = "-";
    else
        signal_y = "+";
    if(velocities[2] < 0)
        signal_z = "-";
    else
        signal_z = "+";
    //Convert the velocities to a string - vx10y20z30
    std::string velocities_string = "vx" + signal_x + std::to_string(x1) + std::to_string(x2) + "y" + signal_y + std::to_string(y1) + std::to_string(y2) + "z" + signal_z + std::to_string(z1) + std::to_string(z2);
    printf("velocities: %s\n", velocities_string.c_str());
    //Write the velocities to the serial port
    char velocities_char[100];
    strcpy(velocities_char, velocities_string.c_str());
    flag_write = 1;
    sWriteDev(&serial_t, velocities_char, sizeof(velocities_char));
    flag_write = 0;
}



int main(int argc, char** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle nh;

    //Subscribe to the waypoint pose
    waypoint_sub = nh.subscribe("waypoint_clicked", 1, waypointCallback);

    //Publish the 3 joint states
    joint_states_pub = nh.advertise<mr_joint_controller::JointTarget>("joint_target", 1);

    //Subscribe to the set velocity
    set_velocity_sub = nh.subscribe("set_velocity", 1, set_velocityCallback);

    //Subscribe to the velocity
    velocity_sub = nh.subscribe("velocity", 1, velocityCallback);

    //Publish the real pose
    position_pub = nh.advertise<geometry_msgs::PointStamped>("real_pose", 1);

    //init serial and connect
    bInitDev(&serial_t);

    bConnect(&serial_t);


    //loop to publish the joint states
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        //read the joint states from the serial port
        read();
        ros::spinOnce();
        loop_rate.sleep();
    }

    vCloseDev(&serial_t);
  
    ros::spin();


}