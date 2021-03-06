cmake_minimum_required(VERSION 2.8.3)
project(simplebot_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  dynamic_reconfigure
)

find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Simplebot.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(simplebot 
  src/simplebot.cpp
  src/simplebot_node.cpp
)

add_dependencies(simplebot 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(simplebot
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS simplebot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY model/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
