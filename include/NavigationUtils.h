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
 * @file NavigationUtils.h
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @version 0.1
 * @date 2021-12-11
 */
#ifndef INCLUDE_NAVIGATIONUTILS_H_
#define INCLUDE_NAVIGATIONUTILS_H_

#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "std_msgs/String.h"
#include "ros/ros.h"

class NavigationUtils {
 public:
    /**
     * @brief Set the Desired Goal 
     * 
     * @param goal 
     * @param position 
     * @param qMsg - Quaternion message
     */
    void setDesiredGoal(move_base_msgs::MoveBaseGoal& goal,
    std::vector<double> position, geometry_msgs::Quaternion& qMsg);

    /**
     * @brief Send the goal to the move_base server
     * 
     * @param goal 
     * @param actionClient 
     */
    bool sendGoal(move_base_msgs::MoveBaseGoal& goal,
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&
    actionClient);

    /**
     * @brief generates a quaternion from r p y 
     * and returns it as a geometry message
     * 
     * @param theta 
     * @return geometry_msgs::Quaternion 
     */
    geometry_msgs::Quaternion convertToQuaternion(double theta);

    /**
     * @brief Checks if the goal has been reached
     * 
     * @param actionClient 
     * @return bool
     */
    bool checkGoalReach(actionlib::SimpleActionClient
    <move_base_msgs::MoveBaseAction>& actionClient);

    /**
     * @brief An emergency stop to halt the robot
     * 
     * @return bool
     */
    bool emergencyStop(actionlib::SimpleActionClient
    <move_base_msgs::MoveBaseAction>& actionClient);

    /**
     * @brief Get the position and yaw values 
     * of each waypoint from a file
     * 
     * @param path 
     * @return std::vector<std::vector<double>> 
     */
    std::vector<std::vector<double>>  getPointsFromFile(std::string path);

    /**
     * @brief Send goal as home position
     * 
     * @param actionClient 
     * @return bool flag 
     */
    bool returnToHome(move_base_msgs::MoveBaseGoal& home,
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&
    actionClient);

    /**
     * @brief Check if all the points in the trajectory 
     *        were reached by the robot
     * 
     * @param success_flags Vector of all the success flags
     * @param dummy_pos vector of the trajectory points
     * @return bool flag to check 
     */
    bool checkTrajectoryCompletion(std::vector<bool>& success_flags,
    std::vector<std::vector<double>>& dummy_pos);

 private:
    bool success_status = false;
    bool emergency_stop = false;
};
#endif  // INCLUDE_NAVIGATIONUTILS_H_
