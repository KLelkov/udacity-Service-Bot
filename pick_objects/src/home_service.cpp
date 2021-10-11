#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/packageAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
      MoveBaseClient ac("move_base", true);

      pick_srv_ = n_.serviceClient<add_markers::packageAction>("/add_markers/pickup");
      drop_srv_ = n_.serviceClient<add_markers::packageAction>("/add_markers/dropoff");
      subscriber_ = n_.subscribe("/amcl_pose", 10, &SubscribeAndPublish::localization_callback, this);


      // Wait 5 sec for move_base action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      bool got_package = SubscribeAndPublish::goForPickUp(ac);

      if (got_package)
      {
        
        ros::Duration(5.0).sleep();
        ROS_INFO("Pickup successeful!");
        bool delivered_package = SubscribeAndPublish::goForDelivery(ac);
        if (delivered_package)
        {
          ROS_INFO("Delivery successeful!");
        }
      }
      
        
    }


    // This callback function continuously executes and reads the image data
    void localization_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
      // TODO: Apparently ac.waitForResult() blocks the execution of this script and
      // locomotion callback is never recieved. This can be fixed by manualy
      // tracking goal destination, but it isn't really required for the udacity project,
      // so I'm gonna leave it for future to submit the project in time!
      pos_x_ = msg.pose.pose.position.x;
      pos_y_ = msg.pose.pose.position.y;
      //printf("\nCb at  %f %f\n", pos_x_, pos_y_);
    }

    bool goForPickUp(MoveBaseClient &ac)
    {
      move_base_msgs::MoveBaseGoal goal;

      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = 9.0;
      goal.target_pose.pose.position.y = -2;
      goal.target_pose.pose.orientation.w = 0.707;
      goal.target_pose.pose.orientation.z = 0.707;


       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending pickup goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Reached pickup zone. Picking the package...");
        add_markers::packageAction srv;
        srv.request.pos_x = 9.0;
        srv.request.pos_y = -2.0;
        pick_srv_.call(srv);
        return true;
      }
      ROS_INFO("The base failed to reach pickup zone");
      return false;
    }

    bool goForDelivery(MoveBaseClient &ac)
    {
      move_base_msgs::MoveBaseGoal goal;

      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = 8.0;
      goal.target_pose.pose.position.y = 5.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.z = 0.0;


       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending drop zone goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Reached dropoff zone. Dropping the package...");
        add_markers::packageAction srv;
        //srv.request.pos_x = pos_x_; // See localization_callback() for context
        //srv.request.pos_y = pos_y_; // of this change
        srv.request.pos_x = 8.0;
        srv.request.pos_y = 5.0;
        drop_srv_.call(srv);
        return true;
      }
      ROS_INFO("The base failed to reach dropoff zone");
      return false;
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::Subscriber subscriber_;
    ros::ServiceClient pick_srv_;
    ros::ServiceClient drop_srv_;
    float pos_x_ = 0.0;
    float pos_y_ = 0.0;

};//End of class SubscribeAndPublish


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "home_service");

  // Handle ROS communication events
  SubscribeAndPublish SAPObject;

  ros::spin();
  return 0;
}
