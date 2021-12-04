
#ifndef INCLUDE_LawnMower_H_
#define INCLUDE_LawnMower_H_
#include <string>
#include <sstream>
#include "ros/ros.h"
#include <NavigationUtils.h>
class LawnMower {
 public:
   
     explicit LawnMower(ros::NodeHandle n, std::string path);
     void mow();
 
 private:
 
     ros::NodeHandle node_h;

     std::string path_to_waypoints;

};
#endif  // INCLUDE_LawnMower_H_