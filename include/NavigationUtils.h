#ifndef INCLUDE_NavigationUtils_H_
#define INCLUDE_NavigationUtils_H_
#include <string>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NavigationUtils { 

    public:
     std::vector<double> getCurrentLocation();

     void setDesiredGoal(move_base_msgs::MoveBaseGoal& goal, std::vector<double> position, geometry_msgs::Quaternion& qMsg);
    
     void sendGoal(move_base_msgs::MoveBaseGoal& goal, 
     actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& actionClient);
    
     geometry_msgs::Quaternion convertToQuaternion(double theta);
    
     bool checkGoalReach(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& actionClient);
    
     bool emergencyStop();
    
     std::vector<std::vector<double>>  getPointsFromFile(std::string path);

     bool returnToHome(move_base_msgs::MoveBaseGoal& goal, 
     actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& actionClient);
};

#endif  // INCLUDE_NavigationUtils_H_