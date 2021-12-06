/**
 * Copyright (c) 2021 Aditi Ramadwar, Arunava Basu
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "NavigationUtils.h"
#include <iostream>
#include <fstream>

std::vector<double> NavigationUtils::getCurrentLocation() {
}

void NavigationUtils::setDesiredGoal(move_base_msgs::MoveBaseGoal& goal,
 std::vector<double> position, geometry_msgs::Quaternion& qMsg) {
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = position[0];
  goal.target_pose.pose.position.y = position[1];
  goal.target_pose.pose.orientation = qMsg;
}

void NavigationUtils::sendGoal(move_base_msgs::MoveBaseGoal& goal,
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& actionClient) {
  actionClient.sendGoal(goal);
}

geometry_msgs::Quaternion NavigationUtils::convertToQuaternion(double theta) {
  double radians = theta*(M_PI/180);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, radians);
  myQuaternion.normalize();
  geometry_msgs::Quaternion qMsg;
  qMsg = tf2::toMsg(myQuaternion);
  return qMsg;
}

bool NavigationUtils::checkGoalReach(actionlib::SimpleActionClient
<move_base_msgs::MoveBaseAction>& actionClient) {
    actionClient.waitForResult();
    bool success = false;
    if (actionClient.getState() ==
    actionlib::SimpleClientGoalState::SUCCEEDED) {
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

std::vector<std::vector<double>>
NavigationUtils::getPointsFromFile(std::string path) {
}

bool NavigationUtils::returnToHome(move_base_msgs::MoveBaseGoal& goal,
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&actionClient) {
}
