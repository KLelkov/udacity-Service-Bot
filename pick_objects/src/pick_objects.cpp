#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.seq = 0;

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -2.0;
  goal.target_pose.pose.position.y = -9;
  goal.target_pose.pose.orientation.w = 0.707;
  goal.target_pose.pose.orientation.z = 0.707;


   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending 1 goal");
  ac.sendGoal(goal);

  

  
  
  bool picked  = true;
  // Wait an infinite time for the results
  ac.waitForResult();
  //ros::Duration(20.0).sleep();
  //ROS_INFO("Cancel the goal");
  //ac.stopTrackingGoal();
  //ac.cancelAllGoals();
  //ros::Duration(15.0).sleep();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
    ROS_INFO("Reached goal 1");
picked = true;
}
  else
    ROS_INFO("The base failed to reach goal 1");
  ros::Duration(5.0).sleep();
  if (picked)
{
  move_base_msgs::MoveBaseGoal goal2;
  goal2.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.seq = 1;


  // Define a position and orientation for the robot to reach
  goal2.target_pose.pose.position.x = 5; // 5
  goal2.target_pose.pose.position.y = -8; // -8
  goal2.target_pose.pose.orientation.w = 1.0;
    //goal.target_pose.pose.orientation.z = 0.707;
  ROS_INFO("Sending 2 goal");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  //ros::Duration(26.0).sleep();
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached goal 2");
  else
    ROS_INFO("The base failed to reach goal 2");
}

  return 0;
}
