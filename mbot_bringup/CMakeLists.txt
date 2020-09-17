cmake_minimum_required(VERSION 2.8.3)
project(mbot_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  move_base_msgs
  actionlib
  visualization_msgs
  cv_bridge
  image_transport
  # eigen
)

set(EIGEN_DIR /usr/include/eigen3)
set(PCL_DIR /home/yat/pcl-pcl-1.10.1/build)
find_package(OpenCV 3.3.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL  REQUIRED COMPONENTS)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# find_package(OpenCV 3.0 QUIET)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mbot_bringup
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

set(OpenCV_DIR /home/yat/opencv-3.3.1/install/share/OpenCV)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /home/yat/AstraSDK--Linux/include
  /home/yat/AstraSDK--Linux/samples/common
  /home/yat/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63/Include
   
  /home/yat/Documents/librealsense2/include 
  /home/yat/Documents/librealsense2/third-party 
  /home/yat/Documents/librealsense2/third-party/tclap/include 
  /home/yat/Documents/librealsense2/wrappers/opencv

)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mbot_bringup 
               src/mbot_bringup.cpp
               src/mbot.cpp)
target_link_libraries(mbot_bringup ${catkin_LIBRARIES})

add_executable(odomtransform
  src/odomtransform.cpp
)
add_dependencies(odomtransform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odomtransform
  ${catkin_LIBRARIES}
)

add_executable(laserdetector
  src/laserdetector.cpp
)
add_dependencies(laserdetector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laserdetector
  ${catkin_LIBRARIES}
)

add_executable(imudata
  src/imudata.cpp
)
add_dependencies(imudata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imudata
  ${catkin_LIBRARIES}
)

add_executable(motion_update
  src/motion_update.cpp
)
add_dependencies(motion_update ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_update
  ${catkin_LIBRARIES}
)

add_executable(test_sonar
  src/test_sonar.cpp
)
add_dependencies(test_sonar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sonar
  ${catkin_LIBRARIES}
)

add_executable(test_collision
  src/test_collision.cpp
)
add_dependencies(test_collision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_collision
  ${catkin_LIBRARIES}
)

add_executable(depthdata
  src/depthdata.cpp
)
add_dependencies(depthdata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depthdata
  ${catkin_LIBRARIES}

  /home/yat/AstraSDK--Linux/lib/libastra.so
  /home/yat/AstraSDK--Linux/lib/libastra_core.so
  /home/yat/AstraSDK--Linux/lib/libastra_core_api.so
  /home/yat/AstraSDK--Linux/lib/Plugins/libopenni_sensor.so
  /home/yat/AstraSDK--Linux/lib/Plugins/libOrbbecBodyTracking.so
  /home/yat/AstraSDK--Linux/lib/Plugins/liborbbec_hand.so
  /home/yat/AstraSDK--Linux/lib/Plugins/liborbbec_xs.so

)

add_executable(test_gohome
  src/test_gohome.cpp
)
add_dependencies(test_gohome ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gohome
  ${catkin_LIBRARIES}
)

add_executable(indoorrobot
  src/WeightedFit.cpp
  src/Automatic_Recharge.cpp
  src/indoorrobot.cpp
)
add_dependencies(indoorrobot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(indoorrobot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

add_executable(moveAlongEdge
  src/moveAlongEdge.cpp
)
add_dependencies(moveAlongEdge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveAlongEdge
  ${catkin_LIBRARIES}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(get_ground
  src/get_ground.cpp
)
add_dependencies(get_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_ground
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}

  /home/yat/Documents/librealsense2/build/librealsense2.so
)


add_executable(calib_plane
  src/calib_plane.cpp
)
add_dependencies(calib_plane ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calib_plane
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}

  /home/yat/Documents/librealsense2/build/librealsense2.so
)

add_executable(planar_segmentation
  src/planar_segmentation_node.cpp
  src/obstacledet.cpp
  src/planar_segmentation.cpp
)
add_dependencies(planar_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planar_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}

  /home/yat/Documents/librealsense2/build/librealsense2.so
)

add_executable(regiongrowing_segmentation
  src/regiongrowing_segmentation.cpp
)
add_dependencies(regiongrowing_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(regiongrowing_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}

  /home/yat/Documents/librealsense2/build/librealsense2.so
)

add_executable(test_get_rgb_line src/rgb_line.cpp test/test_lane_line.cpp) 
target_link_libraries(test_get_rgb_line ${OpenCV_LIBRARIES})

add_executable(movealongedge
  src/move_along_edge_node.cpp
  src/move_along_edge.cpp
)
add_dependencies(movealongedge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(movealongedge
  ${catkin_LIBRARIES}
)

add_executable(image_acquisition
  src/image_proc.cpp
  src/vibe_plus.cpp
)
add_dependencies(image_acquisition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_acquisition
  ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

add_executable(test_laser_recharge
  src/WeightedFit.cpp
  src/laser_recharge.cpp
  test/test_laser_recharge.cpp
)
add_dependencies(test_laser_recharge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_laser_recharge
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)
