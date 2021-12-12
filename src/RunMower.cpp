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
 * @file RunMower.cpp
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @version 0.1
 * @date 2021-12-11
 */
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "LawnMower.h"

/**
 * @brief main function which creates a 
 * new node handle and starts execution
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "alm");
  ros::NodeHandle ros_node_h;
  std::string path =
  "/home/dev_root/catkin_ws/src/Autonomous_Lawn_Mower/data/waypoints.csv";
  ROS_INFO_STREAM("Starting LawnMower... ");
  LawnMower mower(ros_node_h);
  mower.mow(path);
  return 0;
}
