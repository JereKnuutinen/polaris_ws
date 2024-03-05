cmake_minimum_required(VERSION 3.9)
project(ekf_node)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(ALIAS_ROS1_CLASS_LOADER)
    add_definitions(-Dclass_loader=ros1_class_loader)
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  atv_can
  geometry_msgs
  nav_msgs
  sensor_msgs
)
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  atv_can
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  INCLUDE_DIRS include
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 "/codegen/exe"
  "include/ekf_node"
)
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
## Get list of .c files in project source directory
file(GLOB ${PROJECT_NAME}_C_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.c)
## Get list of .cpp files in project source directory
file(GLOB ${PROJECT_NAME}_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.cpp)
# Declare executable - Node
add_executable(EKF_Node
  src/CoderTimeAPI.cpp
  src/EKFCorrectorAdditive.cpp
  src/EKF_Node.cpp
  src/EKF_Node_data.cpp
  src/EKF_Node_initialize.cpp
  src/EKF_Node_rtwutil.cpp
  src/EKF_Node_terminate.cpp
  src/Publisher.cpp
  src/Rate.cpp
  src/Subscriber.cpp
  src/atv_can_OdometryMeasurementStruct.cpp
  src/atv_can_SpeedandSteeringCommandsStruct.cpp
  src/atv_can_SteeringMeasurementStruct.cpp
  src/atv_can_WheelDisplacementMeasurementStruct.cpp
  src/aug_dynamic_model.cpp
  src/cholPSD.cpp
  src/coder_posix_time.c
  src/cosd.cpp
  src/diag.cpp
  src/enu2lla.cpp
  src/eul2quat.cpp
  src/extendedKalmanFilter.cpp
  src/geometry_msgs_PointStruct.cpp
  src/geometry_msgs_PoseStruct.cpp
  src/geometry_msgs_PoseWithCovarianceStruct.cpp
  src/geometry_msgs_QuaternionStruct.cpp
  src/geometry_msgs_TwistStruct.cpp
  src/geometry_msgs_TwistWithCovarianceStruct.cpp
  src/geometry_msgs_Vector3Struct.cpp
  src/lla2enu.cpp
  src/main.cpp
  src/measurement_model2.cpp
  src/mtimes.cpp
  src/nav_msgs_OdometryStruct.cpp
  src/quat2eul.cpp
  src/ros_TimeStruct.cpp
  src/ros_structmsg_conversion.cpp
  src/rtGetInf.cpp
  src/rtGetNaN.cpp
  src/rt_nonfinite.cpp
  src/sensor_msgs_NavSatFixStruct.cpp
  src/sensor_msgs_NavSatStatusStruct.cpp
  src/sind.cpp
  src/std_msgs_HeaderStruct.cpp
  src/svd.cpp
  src/tand.cpp
  src/tic.cpp
  src/toc.cpp
  src/trisolve.cpp
  src/validate_print_arguments.cpp
  src/wrapLatitude.cpp
  src/xaxpy.cpp
  src/xdotc.cpp
  src/xgemv.cpp
  src/xgerc.cpp
  src/xnrm2.cpp
  src/xpotrf.cpp
  src/xrot.cpp
  src/xrotg.cpp
  src/xswap.cpp
  src/intpol_codegen.cpp
  src/myInterpolator2.cpp
  src/mlroscpp_sub.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#target_compile_options(EKF_Node PUBLIC  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>: -Wall -Wextra -Wpedantic >)
endif()
target_compile_options(EKF_Node PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
  -DROS_PROJECT
  -DMW_SCHED_OTHER
  -DSTACK_SIZE=200000
  -DMODEL=EKF_Node
  >
  )
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
  EKF_Node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(EKF_Node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
## Specify libraries to link a library or executable target against
target_link_libraries(
  EKF_Node 
    "/usr/local/MATLAB/R2023a/sys/os/glnxa64/orig/libstdc++.so.6" m
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CMAKE_DL_LIBS}
)
install(TARGETS EKF_Node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET EKF_Node
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/codegen/exe/EKF_Node/${file_i}
				   )
add_custom_command(
                   TARGET EKF_Node
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/EKF_Node_ert_rtw/${file_i}
				   )
endforeach(file_i)
# Generated on Wed Feb 28 10:35:18 2024
