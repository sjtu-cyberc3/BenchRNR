cmake_minimum_required(VERSION 3.0.2)
project(foreground_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
set(OpenCV_DIR "/usr/share/OpenCV/")

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


option(USE_MULTI "Enable MULTI mode" OFF)

if (USE_MULTI)
  add_definitions(-DMULTI)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  livox_ros_driver
  # cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)

find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES MOS_Horizon
#  CATKIN_DEPENDS roscpp stdmsgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

include_directories(${CERES_INCLUDE_DIRS})

include_directories(${roscpp_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/main.cpp src/ikd_Tree.cpp)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
