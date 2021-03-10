CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#############################################
SET(MODULENAME colourToHIS)
#############################################

PROJECT(${MODULENAME})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

FILE(GLOB folder_source *.cpp *.c )
FILE(GLOB folder_header *.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${MODULENAME} ${folder_source} ${folder_header})
 
TARGET_LINK_LIBRARIES(${MODULENAME} ${OpenCV_LIBRARIES} )

INSTALL(TARGETS ${MODULENAME} DESTINATION bin)