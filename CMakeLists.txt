cmake_minimum_required(VERSION 3.0.2)

project(phidgets
  VERSION 1.0.0
  DESCRIPTION ""
  LANGUAGES C CXX
)

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  imu_filter_madgwick
  message_generation
  nodelet
  roscpp
  roslaunch
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

add_message_files(
  FILES
  EncodersT.msg
  DutyCycles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Encoder.cfg
  cfg/Logging.cfg
  cfg/Motor.cfg
  cfg/Spatial.cfg
  cfg/Temperature.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_util phidgets_motor phidgets_motors phidgets_encoder phidgets_encoders phidgets_spatial phidgets_temperature
  CATKIN_DEPENDS message_runtime nodelet dynamic_reconfigure roscpp roslib sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(phidgets_util src/util.cpp src/error.cpp)
add_library(phidgets_logging_nodelet src/logging_nodelet.cpp)
add_library(phidgets_motor src/motor.cpp)
add_library(phidgets_motors src/motors.cpp)
add_library(phidgets_motors_nodelet src/motors_nodelet.cpp)
add_library(phidgets_encoder src/encoder.cpp)
add_library(phidgets_encoders src/encoders.cpp)
add_library(phidgets_encoders_nodelet src/encoders_nodelet.cpp)
add_library(phidgets_spatial src/spatial.cpp)
add_library(phidgets_spatial_nodelet src/spatial_nodelet.cpp)
add_library(phidgets_temperature src/temperature.cpp)
add_library(phidgets_temperature_nodelet src/temperature_nodelet.cpp)

add_dependencies(phidgets_logging_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(phidgets_motor ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_motors ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(phidgets_motors_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_encoders ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(phidgets_encoders_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_spatial ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(phidgets_spatial_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_temperature ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(phidgets_temperature_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_logging_nodelet ${catkin_LIBRARIES} phidget22)
target_link_libraries(phidgets_motor ${catkin_LIBRARIES} phidget22 phidgets_util)
target_link_libraries(phidgets_motors ${catkin_LIBRARIES} phidgets_motor)
target_link_libraries(phidgets_motors_nodelet ${catkin_LIBRARIES} phidgets_motors)
target_link_libraries(phidgets_encoder ${catkin_LIBRARIES} phidget22 phidgets_util)
target_link_libraries(phidgets_encoders ${catkin_LIBRARIES} phidgets_encoder)
target_link_libraries(phidgets_encoders_nodelet ${catkin_LIBRARIES} phidgets_encoders)
target_link_libraries(phidgets_spatial ${catkin_LIBRARIES} phidget22 phidgets_util)
target_link_libraries(phidgets_spatial_nodelet ${catkin_LIBRARIES} phidgets_spatial)
target_link_libraries(phidgets_temperature ${catkin_LIBRARIES} phidget22 phidgets_util)
target_link_libraries(phidgets_temperature_nodelet ${catkin_LIBRARIES} phidgets_temperature)

install(TARGETS phidgets_logging_nodelet phidgets_motor phidgets_motors phidgets_motors_nodelet phidgets_encoder phidgets_encoders phidgets_encoders_nodelet phidgets_spatial phidgets_spatial_nodelet phidgets_temperature phidgets_temperature_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
