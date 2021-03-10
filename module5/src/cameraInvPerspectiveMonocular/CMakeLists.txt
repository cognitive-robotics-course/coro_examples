CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#############################################
SET(MODULENAME cameraInvPerspectiveMonocular)
#############################################

PROJECT(${MODULENAME})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

FILE(GLOB folder_source *.cpp *.c )
FILE(GLOB folder_header ${CMAKE_SOURCE_DIR}/include/module5/${MODULENAME}.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${MODULENAME} ${folder_source} ${folder_header})
 
TARGET_LINK_LIBRARIES(${MODULENAME} ${OpenCV_LIBRARIES} )

INSTALL(TARGETS ${MODULENAME} DESTINATION bin)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
   message(STATUS "Linux detected linking and catkin_LIBRARIES")
   TARGET_LINK_LIBRARIES(${MODULENAME} ${catkin_LIBRARIES})
endif()


