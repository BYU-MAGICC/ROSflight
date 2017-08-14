cmake_minimum_required(VERSION 2.8.3)
project(telemetry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosflight_msgs
  rosplane_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavtelemetry
  CATKIN_DEPENDS roscpp rosflight_msgs rosplane_msgs
  DEPENDS Boost YAML_CPP
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

# mavtelemetry library
add_library(mavtelemetry
  src/mavtelemetry/mavtelemetry.cpp
  src/mavtelemetry/mavlink_serial.cpp
)
add_dependencies(mavtelemetry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavtelemetry
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# telemetry_io_node
add_executable(telemetry_io
  src/telemetry_io_node.cpp
  src/telemetry_io.cpp
)
add_dependencies(telemetry_io ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(telemetry_io
  mavtelemetry
  ${catkin_LIBRARIES}
  ${Boost_LIBRARES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS mavtelemetry telemetry_io
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/telemetry/mavtelemetry/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
