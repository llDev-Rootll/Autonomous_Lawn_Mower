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
 * @file LawnMower.h
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @version 0.1
 * @date 2021-12-11
 */
#ifndef INCLUDE_LAWNMOWER_H_
#define INCLUDE_LAWNMOWER_H_

#include <NavigationUtils.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
 MoveBaseClient;

class LawnMower {
 public:
    /**
     * @brief Construct a new Lawn Mower constructor to 
     * initialize the node handle and path to waypoints file
     * 
     * @param n - node handle
     * @param path - path to waypoints file
     */
    explicit LawnMower(ros::NodeHandle n);

    /**
     * @brief main function which starts the 
     * mowing routine by following the waypoints
     * 
     */
     void mow(std::string path);

     /**
      * @brief Callback function for UI interrupt to start the 
      *        trajectory tracking of robot
      * 
      * @param msg Data containing UI information
      */
     void start(const std_msgs::String& msg);

     /**
      * @brief Callback function for UI interrupt instantly
      *        stop the robot
      * 
      * @param msg Data containing UI information
      */
     void e_stop(const std_msgs::String::ConstPtr& msg);

     /**
      * @brief Callback function for UI interrupt to pause the 
      *        trajectory tracking of robot.
      * 
      * @param msg Data containing UI information
      */
     void pause(const std_msgs::String::ConstPtr& msg);

     /**
      * @brief Callback function for UI interrupt to resume the 
      *        trajectory tracking of robot
      * 
      * @param msg Data containing UI information
      */
     void resume(const std_msgs::String::ConstPtr& msg);

     /**
      * @brief Set the Index o
      * 
      * @param i index of the trajectory
      * @return bool flag for confirmation
      */
     bool setIndex(int i);

     /**
      * @brief Get the Index
      * 
      * @return int index of trajectory vector
      */
     int getIndex();
     std::vector<std::vector<double>> dummy_pos;
     std::vector<bool> success_flags;

 private:
    /**
     * @brief ROS node handle
     * 
     */

    ros::NodeHandle node_h;
    /**
     * @brief path to waypoints file
     * 
     */
    std::string path_to_waypoints;
    move_base_msgs::MoveBaseGoal home;
    MoveBaseClient actionClient;
    std::vector<double> current_goal;
    std::string flag;
    int paused_index = 0;
    bool pause_flag = false;
    
    
};
#endif  // INCLUDE_LAWNMOWER_H_
