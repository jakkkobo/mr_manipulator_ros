/** Description:
* This script is used to define the dynamic reconfigure server for the 3DoF robot arm.
* It creates the following parameters:
* vx, vy, vz: linear velocity in x, y and z axis
* x, y, z: position in x, y and z axis
* stop: stop the robot arm
*/


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mr_dynamic/RobotArmConfig.h>
#include <mr_joint_controller/JointTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>


ros::Publisher velocity_pub;
ros::Publisher set_velocity_pub;
ros::Publisher position_pub;



void callback(mr_dynamic::RobotArmConfig &config, uint32_t level)
{
    //Publish the joint target
    //stop the robot arm
    if(config.EMERGENCY_STOP)
    {
       //send 0,0,0 velocity to the velocity_pub
        geometry_msgs::Vector3 velocity;
        velocity.x = 0;
        velocity.y = 0;
        velocity.z = 0;
        velocity_pub.publish(velocity);
    }
    else if(config.Enable_Velocity_Control && !config.Send_Position)
    {
        //send velocities to the velocity_pub
        geometry_msgs::Vector3 velocity;
        velocity.x = config.linear_velocity_x;
        velocity.y = config.linear_velocity_y;
        velocity.z = config.linear_velocity_z;
        velocity_pub.publish(velocity);
    }
   
    if(!config.Enable_Velocity_Control && !config.EMERGENCY_STOP && !config.Send_Position)
    {
        //send set velocities to set_velocity_pub
        geometry_msgs::Vector3 set_velocity;
        set_velocity.x = config.set_linear_velocity_x;
        set_velocity.y = config.set_linear_velocity_y;
        set_velocity.z = config.set_linear_velocity_z;
        set_velocity_pub.publish(set_velocity);
    }

    if(config.Send_Position)
    {
        //send position to position_pub
        geometry_msgs::PointStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "footprint_link";
        pose.point.x = (double)config.position_x;
        pose.point.y = config.position_y;
        pose.point.z = config.position_z;
        position_pub.publish(pose);
    }


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mr_dynamic_node");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<mr_dynamic::RobotArmConfig> server;
    dynamic_reconfigure::Server<mr_dynamic::RobotArmConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    //Create the publishers
    velocity_pub = nh.advertise<geometry_msgs::Vector3>("velocity", 1);
    set_velocity_pub = nh.advertise<geometry_msgs::Vector3>("set_velocity", 1);
    position_pub = nh.advertise<geometry_msgs::PointStamped>("waypoint_clicked", 1);


    ros::spin();
    return 0;
}