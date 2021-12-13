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
 * @file LawnMower.cpp
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @version 0.1
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2021
 */
#include "LawnMower.h"

/**
 * @brief Callback function for UI interrupt to start the 
 *        trajectory tracking of robot
 * 
 * @param msg Data containing UI information
 */
void LawnMower::start(const std_msgs::String& msg) {
    ROS_DEBUG_STREAM("Successfully received command");
    std::string command = msg.data;
    ROS_INFO_STREAM("UI Message heard : " << command);
    move_base_msgs::MoveBaseGoal goal;
    // MoveBaseClient actionClient("move_base", true);
    while (!actionClient.waitForServer(ros::Duration(5.0))) {
    // Wait for move base server
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    NavigationUtils navUtils;
    geometry_msgs::Quaternion qMsg;
    std::vector<double> element;
    for (int i=getIndex(); i < dummy_pos.size(); i++) {
        // Emergency stop
        if (flag == "e_stop") {
            break;
        }
        if (!pause_flag) {
            if (actionClient.isServerConnected()) {
                setIndex(i+1);
                // convert orietation to quaternion
                element = dummy_pos[i];
                qMsg = navUtils.convertToQuaternion(element[2]);
                // set the goal message with desired postion and orientation
                navUtils.setDesiredGoal(goal, element, qMsg);
                ROS_INFO("Sending goal");
                // send the goal message
                navUtils.sendGoal(goal, actionClient);
                // record the status flag
                success_flags.push_back(navUtils.checkGoalReach(actionClient));
            } else {
                ROS_WARN("Connection to server failed!");
                break;
            }
        } else {
            break;
        }
    }
    // Execute after iterating through the entire trajectory
    if (getIndex() == dummy_pos.size()) {
        if (flag != "e_stop") {
            if (!pause_flag) {
                navUtils.returnToHome(home, actionClient);
                setIndex(0);
            }
        }
    }
    if (navUtils.checkTrajectoryCompletion(success_flags, dummy_pos)) {
      ROS_INFO_STREAM("TRAJECTORY COMPLETED");
    }
}

/**
 * @brief Set the Index o
 * 
 * @param i index of the trajectory
 * @return bool flag for confirmation
 */
bool LawnMower::setIndex(int i) {
    paused_index = i;
    return true;
}

/**
 * @brief Get the Index
 * 
 * @return int index of trajectory vector
 */
int LawnMower::getIndex() {
    return paused_index;
}

/**
 * @brief Callback function for UI interrupt instantly
 *        stop the robot
 * 
 * @param msg Data containing UI information
 */
void LawnMower::e_stop(const std_msgs::String::ConstPtr& msg) {
  ROS_DEBUG_STREAM("Successfully received command");
  std::string command = msg->data.c_str();
  ROS_INFO_STREAM("UI Message heard : " << command);
  NavigationUtils navUtils;
  navUtils.emergencyStop(actionClient);
  flag = "e_stop";
  pause_flag = false;
}

/**
 * @brief Callback function for UI interrupt to pause the 
 *        trajectory tracking of robot.
 * 
 * @param msg Data containing UI information
 */
void LawnMower::pause(const std_msgs::String::ConstPtr& msg) {
  ROS_DEBUG_STREAM("Successfully received command");
  std::string command = msg->data.c_str();
  ROS_INFO_STREAM("UI Message heard : " << command);
  pause_flag = true;
}

/**
 * @brief Callback function for UI interrupt to resume the 
 *        trajectory tracking of robot
 * 
 * @param msg Data containing UI information
 */
void LawnMower::resume(const std_msgs::String::ConstPtr& msg) {
  ROS_DEBUG_STREAM("Successfully received command");
  std::string command = msg->data.c_str();
  ROS_INFO_STREAM("UI Message heard : " << command);
  pause_flag = false;
  std_msgs::String m;
  std::stringstream ss;
  ss << "resume";
  m.data = ss.str();
  start(m);
}

/**
 * @brief The major function which 
 * starts the lawn mowing routine
 * 
 */
void LawnMower::mow(std::string path) {
  NavigationUtils navUtils;
  dummy_pos = navUtils.getPointsFromFile(path);
  ros::Subscriber sub_start = node_h.subscribe("alm_start",
  1000, &LawnMower::start, this);
  ros::Subscriber sub_e_stop = node_h.subscribe("alm_e_stop",
  1000, &LawnMower::e_stop, this);
  ros::Subscriber sub_pause = node_h.subscribe("alm_pause",
  1000, &LawnMower::pause, this);
  ros::Subscriber sub_resume = node_h.subscribe("alm_resume",
  1000, &LawnMower::resume, this);
  // Multi-threading of the four UI nodes
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}

/**
 * @brief Constructor to set the ros 
 * node hand and set the path to waypoints file
 * 
 * @param n 
 * @param path 
 */
LawnMower::LawnMower(ros::NodeHandle n):
actionClient("move_base", true), flag("") {
  node_h = n;
  home.target_pose.header.frame_id = "map";
  home.target_pose.header.stamp = ros::Time::now();
  home.target_pose.pose.position.x = 0.0145;
  home.target_pose.pose.position.y = 0.023;
  home.target_pose.pose.orientation.x = 0;
  home.target_pose.pose.orientation.y = 0;
  home.target_pose.pose.orientation.z = 0.00632866717679;
  home.target_pose.pose.orientation.w = 0.999979973785;
  ros::spinOnce();
}
