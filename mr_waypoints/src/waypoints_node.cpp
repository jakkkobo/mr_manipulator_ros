/* Code to create a matrix of waypoints for the robot to follow
*The waypoint are created in a grid pattern 
*The waypoints are interactive markers that can print out their position
*The waypoints are published to the topic /interactive_waypoints
*The waypoints are published as a PoseArray
*When pressed the waypoints will publish their position to the topic /waypoint_clicked
*/

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>


using namespace visualization_msgs;

ros::Publisher waypoint_clicked_pub;


//Create the interactive marker server
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                    << ", " << feedback->mouse_point.y
                    << ", " << feedback->mouse_point.z
                    << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
        // case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
        // break;

        // case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
        // break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_INFO_STREAM( s.str() << ": pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\norientation = "
            << feedback->pose.orientation.w
            << ", " << feedback->pose.orientation.x
            << ", " << feedback->pose.orientation.y
            << ", " << feedback->pose.orientation.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nsec << " nsec" );
        break;

        // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
        // break;

        // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
        // break;
    }

    //Create the PointStamped to publish the waypoint clicked
    geometry_msgs::PointStamped waypoint_clicked;
    waypoint_clicked.header.frame_id = "footprint_link";
    waypoint_clicked.point.x = feedback->pose.position.x;
    waypoint_clicked.point.y = feedback->pose.position.y;
    waypoint_clicked.point.z = feedback->pose.position.z;

    //Publish the waypoint clicked
    waypoint_clicked_pub.publish(waypoint_clicked);
    server->applyChanges();

}

/*
*Function to create a grid of waypoints
*x, y, z are the starting position of the grid
*x_spacing, y_spacing, z_spacing are the spacing between the waypoints
*x_num, y_num, z_num are the number of waypoints in each direction
*/

void makeWaypointGrid( unsigned int interaction_mode, double x, double y, double z, double x_spacing, double y_spacing, double z_spacing, int x_num, int y_num, int z_num, std::string name)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "footprint_link";
  int_marker.scale = 1.0;

  int_marker.name = name;
  int_marker.description = name;

  InteractiveMarkerControl control;
  uint8_t count = 0;
  //Create the grid of waypoints
  for(int i = 0; i < x_num; i++)
  {
    for(int j = 0; j < y_num; j++)
    {
      for(int k = 0; k < z_num; k++)
      {
        count += 1;
        int_marker.pose.position.x = x + i*x_spacing;
        int_marker.pose.position.y = y + j*y_spacing;
        int_marker.pose.position.z = z + k*z_spacing;
        int_marker.scale = 1.0;

        int_marker.name = name + "_" + std::to_string(count);
        int_marker.description = name + "_" + std::to_string(count);

        InteractiveMarkerControl control;

        if ( interaction_mode == visualization_msgs::InteractiveMarkerControl::BUTTON )
        {
          control.interaction_mode = InteractiveMarkerControl::BUTTON;
          control.name = "button_control";
          Marker marker;
          marker.type = Marker::SPHERE;
          marker.scale.x = 0.02;
          marker.scale.y = 0.02;
          marker.scale.z = 0.02;
          // generate random color
          marker.color.r = 1.0;//random()%255 / 255.0;
          marker.color.g = 1.0;//random()%255 / 255.0;
          marker.color.b = 1.0;//random()%255 / 255.0;
          marker.color.a = 0.1;          
          control.markers.push_back( marker );
          int_marker.controls.push_back( control );

          server->insert(int_marker);
          server->setCallback(int_marker.name, &processFeedback);
        }
        
      }
    }
  }
}

// callback to publish the waypoint clicked
// void clickedWaypoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
// {
//     //Print out the position of the clicked waypoint
//     ROS_INFO("Waypoint clicked at: x = %f, y = %f, z = %f", feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    
//     //Create the PointStamped to publish the waypoint clicked
//     geometry_msgs::PointStamped waypoint_clicked;
//     waypoint_clicked.header.frame_id = "footprint_link";
//     waypoint_clicked.point.x = feedback->pose.position.x;
//     waypoint_clicked.point.y = feedback->pose.position.y;
//     waypoint_clicked.point.z = feedback->pose.position.z;
    
//     //Publish the waypoint clicked
//     waypoint_clicked_pub.publish(waypoint_clicked);
    
// }


int main(int argc, char** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "waypoints_node");
    ros::NodeHandle nh;

    //Publish the waypoint clicked position
    waypoint_clicked_pub = nh.advertise<geometry_msgs::PointStamped>("waypoint_clicked", 1);
    
    //Create the interactive marker server
    server.reset( new interactive_markers::InteractiveMarkerServer("interactive_waypoints","",false));

    ros::Duration(0.1).sleep();
    
    //Create the waypoints
    makeWaypointGrid( visualization_msgs::InteractiveMarkerControl::BUTTON, 0.10, -0.20, 0.05, 0.10, 0.10, 0.10, 3, 5, 3,  "waypoint");
    
    server->applyChanges();

    ros::spin();

    server.reset();
}