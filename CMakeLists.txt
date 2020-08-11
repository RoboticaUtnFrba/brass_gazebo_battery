cmake_minimum_required(VERSION 2.8.3)
project(brass_gazebo_battery)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(gazebo 9 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  gazebo_ros
  roscpp
  message_generation
  gazebo_msgs
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCharge.srv
  SetChargingRate.srv
  SetCharging.srv
  SetCoef.srv
  SetLoad.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    ADD_DEFINITIONS(-fPIC)
ENDIF()

add_library(battery_discharge SHARED src/battery_discharge.cpp src/ROS_debugging.h)
add_dependencies(battery_discharge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_discharge ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(battery_consumer SHARED src/battery_consumer.cpp src/ROS_debugging.h)
add_dependencies(battery_consumer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_consumer ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############

install(TARGETS
        battery_discharge
        battery_consumer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
