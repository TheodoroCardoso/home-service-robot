#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
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

  // Define a position and orientation for the robot to reach
  //goal.target_pose.pose.position.x = 1.0;
  //goal.target_pose.pose.orientation.w = 1.0;

  goal.target_pose.pose.position.x = -6;
  goal.target_pose.pose.position.y = 1.5;
  goal.target_pose.pose.orientation.w = 0.911;
  goal.target_pose.pose.orientation.z = 0.412;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is moving to the pick up zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot picked up virtual object");
  else
    ROS_INFO("Robot failed to move to the pick up zone");

  ros::Duration(5.0).sleep();

  goal.target_pose.pose.position.x = 3;
  goal.target_pose.pose.position.y = -3;
  goal.target_pose.pose.orientation.w = 0.707;
  goal.target_pose.pose.orientation.z = -0.707;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is moving to the drop off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot dropped virtual object");
  else
    ROS_INFO("Robot failed to move to the drop off zone");
  
  ros::spin(); 
  return 0;
}
