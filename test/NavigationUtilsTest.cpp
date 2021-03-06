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
 * 
 * @file NavigationUtilsTest.cpp
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @brief File for testing the NavigationUtils class
 * @version 0.1
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2021
 */
#include <gtest/gtest.h>
#include "NavigationUtils.h"

/**
 * @brief Unit test for convertToQuaternion method
 * 
 */
TEST(NavigationUtilsTests, convert_To_Quaternion) {
  NavigationUtils nav_test;
  geometry_msgs::Quaternion q_test;
  q_test = nav_test.convertToQuaternion(0.0);
  EXPECT_EQ(1, q_test.w);
}

/**
 * @brief Unit test for setDesiredGoal method
 * 
 */
TEST(NavigationUtilsTests, set_desired_goal) {
  NavigationUtils nav_test;
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::Quaternion quart;
  quart.x = 0;
  quart.y = 0;
  quart.z = 0;
  quart.w = 0;
  std::vector<double> position = {0.5, 0};
  nav_test.setDesiredGoal(goal, position, quart);
  EXPECT_EQ(position[0], goal.target_pose.pose.position.x);
}

/**
 * @brief Unit test for sendGoal method
 * 
 */
TEST(NavigationUtilsTests, send_goal_to_server) {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  MoveBaseClient;
  NavigationUtils nav_test;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.1;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  MoveBaseClient actionClient("move_base", true);
  while (!actionClient.waitForServer(ros::Duration(1.0))) {
    // Wait for move base server
    ros::spinOnce();
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  EXPECT_TRUE(nav_test.sendGoal(goal, actionClient));
}

/**
 * @brief Unit test for checkGoalReach method
 * 
 */
TEST(NavigationUtilsTests, check_if_Goal_Reached) {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  MoveBaseClient;
  NavigationUtils nav_test;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.1;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  MoveBaseClient actionClient("move_base", true);
  while (!actionClient.waitForServer(ros::Duration(1.0))) {
    // Wait for move base server
    ros::spinOnce();
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  actionClient.sendGoal(goal);
  EXPECT_TRUE(nav_test.checkGoalReach(actionClient));
}

/**
 * @brief Unit test for checkTrajectoryCompletion method
 * 
 */
TEST(NavigationUtilsTests, check_traj_completion) {
  NavigationUtils nav_test;
  std::vector<bool> success_flags_test = {true, true};
  std::vector<std::vector<double>> dummy_pos_test = {{0, 0, 0}, {0, 0, 0}};
  EXPECT_TRUE(nav_test.checkTrajectoryCompletion(success_flags_test,
  dummy_pos_test));
}
