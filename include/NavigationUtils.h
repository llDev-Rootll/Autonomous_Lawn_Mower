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
#ifndef INCLUDE_NAVIGATIONUTILS_H_
#define INCLUDE_NAVIGATIONUTILS_H_

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <sstream>
#include <vector>
#include "ros/ros.h"

class NavigationUtils {
 public:
    std::vector<double> getCurrentLocation();

    void setDesiredGoal(move_base_msgs::MoveBaseGoal& goal,
    std::vector<double> position, geometry_msgs::Quaternion& qMsg);

    void sendGoal(move_base_msgs::MoveBaseGoal& goal,
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&
    actionClient);

    geometry_msgs::Quaternion convertToQuaternion(double theta);

    bool checkGoalReach(actionlib::SimpleActionClient
    <move_base_msgs::MoveBaseAction>& actionClient);

    bool emergencyStop();

    std::vector<std::vector<double>>  getPointsFromFile(std::string path);

    bool returnToHome(move_base_msgs::MoveBaseGoal& goal
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>&
    actionClient);
};
#endif  // INCLUDE_NAVIGATIONUTILS_H_
