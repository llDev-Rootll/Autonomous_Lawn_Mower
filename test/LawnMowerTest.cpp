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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sstream>
#include "LawnMower.h"
#include "NavigationUtils.h"
#include "std_msgs/String.h"

/**
 * @brief Unit test for getIndex, setIndex method
 * 
 */
TEST(LawnMowerTests, test_index_setter_getter) {
  ros::NodeHandle nh;
  LawnMower mower_test(nh);
  EXPECT_TRUE(mower_test.setIndex(2));
  EXPECT_EQ(2, mower_test.getIndex());
}

TEST(LawnMowerTests, test_mowing_functionality) {
 
  ros::NodeHandle ros_node_h;
  std::string path = "/home/dev_root/catkin_ws/src/Autonomous_Lawn_Mower/data/waypoints_test.csv";
  NavigationUtils navUtils;
  LawnMower mower(ros_node_h);
  mower.dummy_pos = navUtils.getPointsFromFile(path);
  std_msgs::String msg;
  std::stringstream ss;
  ss << "start";
  msg.data = ss.str();
  mower.start(msg);
  while(true) {
    if(mower.dummy_pos.size()==mower.success_flags.size()){
      EXPECT_TRUE(true);
      break;
    }
  }
  

}

