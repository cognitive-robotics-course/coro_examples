cmake_minimum_required(VERSION 3.0.2)
project(module4)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  roslib
  lynxmotion_al5d_description
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(${PROJECT_NAME}_robotProgramming src/robotProgrammingApplication.cpp  src/robotProgrammingImplementation.cpp  src/lynxmotionUtilities.cpp)
set_target_properties(${PROJECT_NAME}_robotProgramming PROPERTIES OUTPUT_NAME robotProgramming  PREFIX "")

add_executable(${PROJECT_NAME}_pickAndPlace src/pickAndPlaceApplication.cpp src/pickAndPlaceImplementation.cpp  src/lynxmotionUtilities.cpp)
set_target_properties(${PROJECT_NAME}_pickAndPlace PROPERTIES OUTPUT_NAME pickAndPlace  PREFIX "")

# Install data files
install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

target_link_libraries(${PROJECT_NAME}_robotProgramming ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_pickAndPlace ${catkin_LIBRARIES})
