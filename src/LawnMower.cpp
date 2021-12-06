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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "LawnMower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
 MoveBaseClient;

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
