cmake_minimum_required(VERSION 3.0.2)
project(alm)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf
  std_msgs
)
catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mow src/LawnMower.cpp src/NavigationUtils.cpp)
target_link_libraries(mow ${catkin_LIBRARIES})



