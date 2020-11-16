cmake_minimum_required (VERSION 2.6)

project (closed_chain_motion_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
LIST( APPEND CMAKE_MODULE_PATH "$ENV{HOME}/cmake" )

add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_simple_controller_manager
  moveit_planners_ompl
  moveit_visual_tools
  trajectory_msgs
  trac_ik_lib
  kdl_parser
  tf2
  tf2_ros
  eigenpy
  roslib
)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
# find_package(Boost COMPONENTS program_options REQUIRED system)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(shape_msgs REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core 
    moveit_ros_planning_interface
    moveit_simple_controller_manager
    rviz_visual_tools
    
  LIBRARIES 
    ${PROJECT_NAME} 
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
    ${catkin_INCLUDE_DIRS}
    # "/home/jiyeong/catkin_ws/devel/include" 
    ${OMPL_INCLUDE_DIRS}
  DEPENDS
    EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}    
    /usr/include/eigen3 
    /usr/include/boost 
    /usr/local/include/ompl
    ${RBDL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${Boost_LIBRARY_DIR})

set(SOURCES
  src/base/jy_ProjectedStateSpace.cpp
  src/base/constraints/ConstrainedPlanningCommon.cpp
  # src/base/constraints/ik_constraint.cpp
  src/base/constraints/ik_task.cpp
  # src/base/cost/ik_cost.cpp
  src/planner/stefanBiPRM.cpp
  src/kinematics/panda_tracik.cpp
  src/kinematics/panda_rbdl.cpp
  src/kinematics/grasping_point.cpp
  src/kinematics/KinematicChain.cpp
  )

add_library(${PROJECT_NAME}_lib
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}  
  ${PCL_LIBRARIES}
  rbdl
  ${OMPL_LIBRARIES}
  yaml-cpp
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries (${PROJECT_NAME}              
  ${catkin_LIBRARIES}   
  ${PROJECT_NAME}_lib       
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  rbdl
  fcl
  yaml-cpp
)

                                
                                        
install(TARGETS ${PROJECT_NAME}_lib
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
