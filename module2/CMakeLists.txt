cmake_minimum_required(VERSION 2.8.3)
project(module2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  turtlesim
)

catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_hello src/hello.cpp)
set_target_properties(${PROJECT_NAME}_hello PROPERTIES OUTPUT_NAME hello  PREFIX "")
target_link_libraries(${PROJECT_NAME}_hello ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pubvel src/pubvel.cpp)
set_target_properties(${PROJECT_NAME}_pubvel PROPERTIES OUTPUT_NAME pubvel  PREFIX "")
target_link_libraries(${PROJECT_NAME}_pubvel ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_subpose src/subpose.cpp)
set_target_properties(${PROJECT_NAME}_subpose PROPERTIES OUTPUT_NAME subpose  PREFIX "")
target_link_libraries(${PROJECT_NAME}_subpose ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_useservices src/useservices.cpp)
set_target_properties(${PROJECT_NAME}_useservices PROPERTIES OUTPUT_NAME useservices  PREFIX "")
target_link_libraries(${PROJECT_NAME}_useservices ${catkin_LIBRARIES})
