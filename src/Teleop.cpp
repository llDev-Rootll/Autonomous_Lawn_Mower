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
 * @file Teleop.cpp
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @author Arunava Basu (arunava@umd.edu)
 * @brief File for publishing messages corresponding to the key triggered on the 
 *        keyboard.
 * @version 0.1
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2021
 */
#include "Teleop.h"

int Teleop::getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);  // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);  // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();  // read character (non-blocking)
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle n;
    ros::Publisher alm_start = n.advertise<std_msgs::String>
    ("alm_start", 1000);
    ros::Publisher alm_pause = n.advertise<std_msgs::String>
    ("alm_pause", 1000);
    ros::Publisher alm_resume = n.advertise<std_msgs::String>
    ("alm_resume", 1000);
    ros::Publisher alm_e_stop = n.advertise<std_msgs::String>
    ("alm_e_stop", 1000);
    Teleop tel;
    ros::Rate loop_rate(10);
    std::cout << "Reading from keyboard" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "S - Start, P - Pause, R - Resume, E- Emergency Stop"
    << std::endl;
    std::cout << "---------------------------" << std::endl;
    while (ros::ok()) {
        int c = tel.getch();   // call non-blocking input function
        if (c == 's') {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "start";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            alm_start.publish(msg);
        }
        if (c == 'p') {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "pause";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            alm_pause.publish(msg);
        }
        if (c == 'r') {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "resume";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            alm_resume.publish(msg);
        }
        if (c == 'e') {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "e_stop";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            alm_e_stop.publish(msg);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
