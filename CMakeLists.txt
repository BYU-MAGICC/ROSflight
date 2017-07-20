cmake_minimum_required(VERSION 2.8.3)
project(telemetry)

# Use this to make Eigen run REALLY fast
set(CMAKE_CXX_FLAGS "-std=c++0x -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosflight_msgs
  rosplane_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Look for and clone MAVLINK if it is missing
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/telemetry/mavlink/v1.0/.git")
  message(STATUS "MAVLink submodule not found at ${CMAKE_CURRENT_SOURCE_DIR}/include/telemetry/mavlink/v1.0")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavtelemetry
  CATKIN_DEPENDS roscpp geometry_msgs rosflight_msgs rosplane_msgs sensor_msgs std_msgs
  DEPENDS Boost EIGEN3 YAML_CPP
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

# mavtelemetry library
add_library(mavtelemetry
  src/mavtelemetry/mavtelemetry.cpp
  src/mavtelemetry/mavlink_serial.cpp
  src/mavtelemetry/param_manager.cpp
  src/mavtelemetry/param.cpp
  src/mavtelemetry/time_manager.cpp
  src/mavtelemetry/sensors/imu.cpp
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
