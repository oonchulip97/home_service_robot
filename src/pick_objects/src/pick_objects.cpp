#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define a vector to store multiple goals
std::vector<std::vector<float> > goals_vector{
{-2.5, -3.5, 0.0, 0.0, 0.0, 0.0, 1.0},
{6.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0},
};

int main(int argc, char** argv){
  // Initialize the node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  for (auto current_goal: goals_vector){
    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = current_goal.at(0);
    goal.target_pose.pose.position.y = current_goal.at(1);
    goal.target_pose.pose.position.z = current_goal.at(2);
    goal.target_pose.pose.orientation.x = current_goal.at(3);
    goal.target_pose.pose.orientation.y = current_goal.at(4);
    goal.target_pose.pose.orientation.z = current_goal.at(5);
    goal.target_pose.pose.orientation.w = current_goal.at(6);

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal to robot");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ROS_INFO("Waiting for robot to arrive at goal");
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot reached the goal");
    else
      ROS_INFO("The robot failed in reaching the goal");

    // Pause for a few seconds
    sleep(5);
  }

  return 0;
}
