cmake_minimum_required(VERSION 2.8.3)
project(module3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Install data files
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

add_executable       (${PROJECT_NAME}_goToPosition src/goToPositionImplementation.cpp src/goToPositionApplication.cpp)
set_target_properties(${PROJECT_NAME}_goToPosition PROPERTIES OUTPUT_NAME goToPosition  PREFIX "")
target_link_libraries(${PROJECT_NAME}_goToPosition ${catkin_LIBRARIES})

add_executable       (${PROJECT_NAME}_goToPoseCreate src/goToPoseCreateImplementation.cpp src/goToPoseCreateApplication.cpp)
set_target_properties(${PROJECT_NAME}_goToPoseCreate PROPERTIES OUTPUT_NAME goToPoseCreate  PREFIX "")
target_link_libraries(${PROJECT_NAME}_goToPoseCreate ${catkin_LIBRARIES})
