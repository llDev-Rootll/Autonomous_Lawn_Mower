#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "LawnMower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
 MoveBaseClient;


// int main(int argc, char** argv){
//   ros::init(argc, argv, "simple_navigation_goals");

//   //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);

//   //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//   //we'll send a goal to the robot to move 1 meter forward
//   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.stamp = ros::Time::now();

//   goal.target_pose.pose.position.x = 1.0;
//   goal.target_pose.pose.orientation.w = 1.0;

//   ROS_INFO("Sending goal");
//   ac.sendGoal(goal);

//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Hooray, the base moved 1 meter forward");
//   else
//     ROS_INFO("The base failed to move forward 1 meter for some reason");

//   return 0;
// }
void LawnMower::mow() {

  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient actionClient("move_base", true);
  while (!actionClient.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  NavigationUtils navUtils;
  geometry_msgs::Quaternion qMsg;
  std::vector<std::vector<double>> dummy_pos = {{0.5, 0, 0},
  {0.5, 0, 0}, {0, 0, 90}, {0.5, 0, 0}, {0.5, 0, 0} };
  for (auto & element : dummy_pos) {
    qMsg = navUtils.convertToQuaternion(element[2]);
    navUtils.setDesiredGoal(goal, element, qMsg);
    ROS_INFO("Sending goal");
    navUtils.sendGoal(goal, actionClient);
    bool success_flag = navUtils.checkGoalReach(actionClient);
  }

}
LawnMower::LawnMower(ros::NodeHandle n, std::string path) {
  node_h = n;
  path_to_waypoints = path;
  ros::spinOnce();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alm");
  ros::NodeHandle ros_node_h;
  std::string path = "../data/waypoints.txt";
  ROS_INFO_STREAM("Starting LawnMower... ");
  LawnMower mower(ros_node_h, path);
  mower.mow();
  ros::spin();
  return 0;
}