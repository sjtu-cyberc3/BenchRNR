cmake_minimum_required(VERSION 3.0.2)
project(clustering_and_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

if (BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(USER_NAME "zrx")
if(USER_NAME STREQUAL "zrx")
    set(livox_ros_driver_DIR "/home/runrunxin/packages/ws_livox_ros_driver/devel/share/livox_ros_driver/cmake")
elseif(USER_NAME STREQUAL "hyf")
    set(livox_ros_driver_DIR "/home/hyf/packages/livox/ws_livox/devel/share/livox_ros_driver/cmake")
    set(Ceres_DIR "/home/hyf/lib/ceres-solver_install/lib/cmake/Ceres")
elseif(USER_NAME STREQUAL "nuc1")
    set(livox_ros_driver_DIR "/home/cyber/packages/livox_ros_driver-master/devel/share/livox_ros_driver/cmake")
elseif(USER_NAME STREQUAL "nuc2")
    set(livox_ros_driver_DIR "/home/cyber/livox/ws_livox_ros_driver/devel/share/livox_ros_driver/cmake")
    add_definitions(-DUSE_MID360=1)
elseif(USER_NAME STREQUAL "nuc0")
    set(livox_ros_driver_DIR "/home/cyber/ws_livox/devel/share/livox_ros_driver/cmake")
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  livox_ros_driver
  # cv_bridge
  # image_transport
  message_filters
  message_generation
  roscpp
  pcl_ros
)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)

find_package(PROJ REQUIRED)


find_package(Ceres REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES MOS_Horizon
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PROJ_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})


option(USE_MULTI "Enable MULTI mode" OFF)

if (USE_MULTI)
  add_definitions(-DMULTI)
endif()

add_executable(${PROJECT_NAME}_node src/main.cpp src/ceres_icp.cpp src/myhash.cpp src/tracker.cpp src/utilis.cpp src/Hungarian.cpp src/clustering.cpp
        include/convex_optimize_fitting.h
        src/convex_optimize_fitting.cpp)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PROJ_LIBRARIES}
)
