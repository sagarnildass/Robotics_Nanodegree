#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //set up publisher to broadcast if robot is at pick up location
  ros::Publisher location_pub = n.advertise<std_msgs::UInt8>("/destination_reached", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal2;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach at each goal
  goal.target_pose.pose.position.x = 3.95;
  goal.target_pose.pose.position.y = 7.5;
  goal.target_pose.pose.orientation.w = 1.0;
  goal2.target_pose.pose.position.x = -3.5;
  goal2.target_pose.pose.position.y = 5.0;
  goal2.target_pose.pose.orientation.w = 1.0;

   // Send the first goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reached the first zone");

    //Notify the add_markers_node that the first goal has been reached
    std_msgs::UInt8 msg;
    msg.data = 1;
    ROS_INFO("The message is %d", msg.data);
    location_pub.publish(msg);

    //Wait for 5 seconds before continuing to next goal
    sleep(5);
  }
  else
  {
    ROS_INFO("The robot failed to reach the first zone");
    return 0;
  }
  
  // Reached first goal, let's move on!
  ROS_INFO("Sending the second goal!");
  ac.sendGoal(goal2);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reached both zones");

    //Notify the add_markers_node that the second goal has been reached
    std_msgs::UInt8 msg2;
    msg2.data = 3;
    ROS_INFO("The message is %d", msg2.data);
    location_pub.publish(msg2);
  }
  else
  {
    ROS_INFO("The robot failed to reach the second zone");
  }

  sleep(5);
  
  return 0;
}