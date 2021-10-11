#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/packageAction.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
      marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      subscriber_ = n_.subscribe("/amcl_pose", 10, &SubscribeAndPublish::localization_callback, this);

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
      marker.pose.position.x = 9.0;
      marker.pose.position.y = -2.0;
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
      if ( fabs(req.pos_x - pos_x_) <= 0.15 && fabs(req.pos_y - pos_y_) <= 0.15)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "add_markers";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.pose.position.x = pos_x_;
        marker.pose.position.y = pos_y_;
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
      else
      {
        ROS_INFO("The robot is not near the package to pick it up!");
        res.result = false;
        return false;
      }
    }

    bool handle_dropoff_request(add_markers::packageAction::Request& req, add_markers::packageAction::Response& res)
    {
      if ( fabs(req.pos_x - pos_x_) <= 0.15 && fabs(req.pos_y - pos_y_) <= 0.15)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "add_markers";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos_x_;
        marker.pose.position.y = pos_y_;
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
      else
      {
        ROS_INFO("The robot has requested the dropoff at the different location. Aborting...");
        res.result = false;
        return false;
      }
    }

    // This callback function continuously executes and reads the image data
    void localization_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
      pos_x_ = msg.pose.pose.position.x;
      pos_y_ = msg.pose.pose.position.y;
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::ServiceServer pickup_;
    ros::ServiceServer dropoff_;
    ros::Publisher marker_pub_;
    ros::Subscriber subscriber_;
    float pos_x_ = 0.0;
    float pos_y_ = 0.0;
  
};//End of class SubscribeAndPublish


int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers_srv");

  // Handle ROS communication events
  SubscribeAndPublish SAPObject;

  ros::spin();
  return 0;
}
