cmake_minimum_required(VERSION 3.9)
project(nmpc_node)
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
  "include/nmpc_node"
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
add_executable(NMPC_Node
  src/BFGSUpdate.cpp
  src/CoderTimeAPI.cpp
  src/NMPC_Node.cpp
  src/NMPC_Node_data.cpp
  src/NMPC_Node_initialize.cpp
  src/NMPC_Node_rtwutil.cpp
  src/NMPC_Node_terminate.cpp
  src/PresolveWorkingSet.cpp
  src/Publisher.cpp
  src/Rate.cpp
  src/ServiceClient.cpp
  src/Subscriber.cpp
  src/addAeqConstr.cpp
  src/addBoundToActiveSetMatrix_.cpp
  src/all.cpp
  src/atv_can_DriveServiceRequestStruct.cpp
  src/atv_can_DriveServiceResponseStruct.cpp
  src/checkStoppingAndUpdateFval.cpp
  src/checkVectorNonFinite.cpp
  src/coder_posix_time.c
  src/computeComplError.cpp
  src/computeFval.cpp
  src/computeFval_ReuseHx.cpp
  src/computeGradLag.cpp
  src/computeGrad_StoreHx.cpp
  src/computeLambdaLSQ.cpp
  src/computeLinearResiduals.cpp
  src/computeMeritFcn.cpp
  src/computePrimalFeasError.cpp
  src/computeQ_.cpp
  src/compute_deltax.cpp
  src/countsort.cpp
  src/custom_cost_dynamic.cpp
  src/deleteColMoveEnd.cpp
  src/driver.cpp
  src/driver1.cpp
  src/dynamic_model_nmpc.cpp
  src/evalObjAndConstr.cpp
  src/evalObjAndConstrAndDerivatives.cpp
  src/factorQR.cpp
  src/factoryConstruct.cpp
  src/factoryConstruct1.cpp
  src/factoryConstruct2.cpp
  src/feasibleX0ForWorkingSet.cpp
  src/feasibleratiotest.cpp
  src/fmincon.cpp
  src/fullColLDL2_.cpp
  src/geometry_msgs_AccelStruct.cpp
  src/geometry_msgs_PointStruct.cpp
  src/geometry_msgs_PoseStruct.cpp
  src/geometry_msgs_PoseWithCovarianceStruct.cpp
  src/geometry_msgs_QuaternionStruct.cpp
  src/geometry_msgs_TwistStruct.cpp
  src/geometry_msgs_TwistWithCovarianceStruct.cpp
  src/geometry_msgs_Vector3Struct.cpp
  src/initActiveSet.cpp
  src/iterate.cpp
  src/linearForm_.cpp
  src/main.cpp
  src/maxConstraintViolation.cpp
  src/modifyOverheadPhaseOne_.cpp
  src/nav_msgs_OdometryStruct.cpp
  src/nlmpcmoveCodeGeneration.cpp
  src/partialColLDL3_.cpp
  src/printInfo.cpp
  src/ratiotest.cpp
  src/relaxed.cpp
  src/removeConstr.cpp
  src/ros_TimeStruct.cpp
  src/ros_structmsg_conversion.cpp
  src/rtGetInf.cpp
  src/rtGetNaN.cpp
  src/rt_nonfinite.cpp
  src/saveJacobian.cpp
  src/setProblemType.cpp
  src/soc.cpp
  src/solve.cpp
  src/sortLambdaQP.cpp
  src/squareQ_appendCol.cpp
  src/std_msgs_HeaderStruct.cpp
  src/step.cpp
  src/test_exit.cpp
  src/tic.cpp
  src/toc.cpp
  src/updateWorkingSetForNewQP.cpp
  src/xgemm.cpp
  src/xgemv.cpp
  src/xgeqp3.cpp
  src/xgerc.cpp
  src/xnrm2.cpp
  src/xpotrf.cpp
  src/xrotg.cpp
  src/xzgeqp3.cpp
  src/xzlarf.cpp
  src/xzlarfg.cpp
  src/znlmpc_computeJacobianState.cpp
  src/znlmpc_getUBounds.cpp
  src/mlroscpp_sub.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#target_compile_options(NMPC_Node PUBLIC  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>: -Wall -Wextra -Wpedantic >)
endif()
target_compile_options(NMPC_Node PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
  -DROS_PROJECT
  -DMW_SCHED_OTHER
  -DSTACK_SIZE=200000
  -DMODEL=NMPC_Node
  >
  )
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
  NMPC_Node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(NMPC_Node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
## Specify libraries to link a library or executable target against
target_link_libraries(
  NMPC_Node 
    "/usr/local/MATLAB/R2023a/sys/os/glnxa64/orig/libstdc++.so.6" m
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CMAKE_DL_LIBS}
)
install(TARGETS NMPC_Node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET NMPC_Node
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/codegen/exe/NMPC_Node/${file_i}
				   )
add_custom_command(
                   TARGET NMPC_Node
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CATKIN_DEVEL_PREFIX}/NMPC_Node_ert_rtw/${file_i}
				   )
endforeach(file_i)
# Generated on Mon Jul 31 12:53:04 2023
