#include "NavigationUtils.h"
#include <iostream>
#include <fstream>

std::vector<double> NavigationUtils::getCurrentLocation() {

}

void NavigationUtils::setDesiredGoal(move_base_msgs::MoveBaseGoal goal, std::vector<double>) {


}

void NavigationUtils::sendGoal(move_base_msgs::MoveBaseGoal goal, 
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {

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

}

bool NavigationUtils::emergencyStop() {

}

std::vector<std::vector<double>>  NavigationUtils::getPointsFromFile(std::string path) {

}

bool NavigationUtils::returnToHome(move_base_msgs::MoveBaseGoal goal, 
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {

}