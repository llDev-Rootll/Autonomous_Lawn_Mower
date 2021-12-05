#include "NavigationUtils.h"
#include <iostream>
#include <fstream>

std::vector<double> NavigationUtils::getCurrentLocation() {

}

void NavigationUtils::setDesiredGoal(move_base_msgs::MoveBaseGoal goal, std::vector<double> position, geometry_msgs::Quaternion qMsg) {
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = position[0];
  goal.target_pose.pose.position.y = position[1];
  goal.target_pose.pose.orientation = qMsg;
}

void NavigationUtils::sendGoal(move_base_msgs::MoveBaseGoal goal, 
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {
  ROS_INFO("Move robot: x= %f, y= %f, yaw= %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation);
  actionClient.sendGoal(goal);
}

geometry_msgs::Quaternion NavigationUtils::convertToQuaternion(double theta) {
  double radians = theta*(M_PI/180);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quaternion, qMsg);
  return qMsg;
}

bool NavigationUtils::checkGoalReach(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {
 actionClient.waitForResult();
 bool success = false;
    if (actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("You have reached goal..");
      success = true;
    } else {
      ROS_INFO("The base failed to reach goal..");
      success = false;
    }
    return success;
}

bool NavigationUtils::emergencyStop() {

}

std::vector<std::vector<double>>  NavigationUtils::getPointsFromFile(std::string path) {

}

bool NavigationUtils::returnToHome(move_base_msgs::MoveBaseGoal goal, 
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {

}