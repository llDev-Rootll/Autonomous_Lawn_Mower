cmake_minimum_required(VERSION 3.0.2)
project(alm)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf2
  tf2_ros
  std_msgs
)
catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mow src/LawnMower.cpp src/NavigationUtils.cpp src/RunMower.cpp)
add_dependencies(mow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mow ${catkin_LIBRARIES})

add_executable(teleop src/Teleop.cpp )
add_dependencies(teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop ${catkin_LIBRARIES})

#Testing block for rostest
if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 add_rostest_gtest(test_mow test/test.launch test/main.cpp src/NavigationUtils.cpp 
                src/LawnMower.cpp test/NavigationUtilsTest.cpp test/LawnMowerTest.cpp)
 target_link_libraries(test_mow ${catkin_LIBRARIES})

endif()
