#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/packageAction.h"

class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
      marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

      // Define a pickup_ service with a pickup_request callback function
      pickup_ = n_.advertiseService("/add_markers/pickup", &SubscribeAndPublish::handle_pickup_request, this);

      // Define a dropoff_ service with a dropoff_request callback function
      dropoff_ = n_.advertiseService("/add_markers/dropoff", &SubscribeAndPublish::handle_dropoff_request, this);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "add_markers";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = -2;
      marker.pose.position.y = -9;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();
      ros::Duration(1.0).sleep();
      marker_pub_.publish(marker);
    }


    bool handle_pickup_request(add_markers::packageAction::Request& req, add_markers::packageAction::Response& res)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "add_markers";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = req.pos_x;
      marker.pose.position.y = req.pos_y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();
      marker_pub_.publish(marker);

      // Wait for a bit
      ros::Duration(1).sleep();

      // Return a response message
      res.result = true;

      return true;
    }

    bool handle_dropoff_request(add_markers::packageAction::Request& req, add_markers::packageAction::Response& res)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "add_markers";
      marker.id = 2;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = req.pos_x;
      marker.pose.position.y = req.pos_y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();
      marker_pub_.publish(marker);
      
      // Wait for a bit
      ros::Duration(1).sleep();

      // Return a response message
      res.result = true;

      return true;
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::ServiceServer pickup_;
    ros::ServiceServer dropoff_;
    ros::Publisher marker_pub_;
  
};//End of class SubscribeAndPublish


int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers_srv");

  // Handle ROS communication events
  SubscribeAndPublish SAPObject;

  ros::spin();
  return 0;
}