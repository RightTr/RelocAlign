cmake_minimum_required(VERSION 3.0.2)
project(relocalign)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set(ROS_EDITION "" CACHE STRING "ROS edition: ROS1 or ROS2")

option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" OFF)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(catkin_FOUND)
  message(STATUS "ROS1 detected")
  set(USE_ROS1 ON)
elseif(ament_cmake_FOUND)
  message(STATUS "ROS2 detected")
  set(USE_ROS2 ON)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(src/fast_gicp)

if(BUILD_VGICP_CUDA)
  add_definitions(-DUSE_VGICP_CUDA)
endif()

if(USE_ROS2)
  add_definitions(-DUSE_ROS2)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)

  add_executable(reloc_manual_xyzquat src/manual/reloc_manual_xyzquat.cpp)
  ament_target_dependencies(reloc_manual_xyzquat
    rclcpp
    geometry_msgs
  )

  add_executable(reloc_manual_xyzyaw src/manual/reloc_manual_xyzyaw.cpp)
  ament_target_dependencies(reloc_manual_xyzyaw
    rclcpp
    geometry_msgs
  )

  add_executable(reloc_livox src/reloc_livox.cpp src/relocalign.cpp)
  target_include_directories(reloc_livox
    PRIVATE
      include
      ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(reloc_livox
    rclcpp
    geometry_msgs
    sensor_msgs
    pcl_conversions
    livox_ros_driver2
  )
  target_link_libraries(reloc_livox
    PRIVATE
      fast_gicp
      yaml-cpp
      ${PCL_LIBRARIES}
  )

  add_executable(demo_reloc src/demo_reloc.cpp src/relocalign.cpp)
  target_include_directories(demo_reloc
    PRIVATE
      include
      ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(demo_reloc
    PRIVATE
      fast_gicp
      yaml-cpp
      ${PCL_LIBRARIES}
  )

  add_executable(cloud_collect src/cloud_collect.cpp)
  target_include_directories(cloud_collect
    PRIVATE
      include
      ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(cloud_collect
    rclcpp
    sensor_msgs
    pcl_conversions
  )
  target_link_libraries(cloud_collect
    PRIVATE
      ${PCL_LIBRARIES}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(TARGETS
    reloc_manual_xyzquat
    reloc_manual_xyzyaw
    reloc_livox
    demo_reloc
    cloud_collect
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
else()
  add_definitions(-DUSE_ROS1)

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    pcl_conversions
    livox_ros_driver2
  )

  catkin_package(
    CATKIN_DEPENDS
      geometry_msgs
      roscpp
      sensor_msgs
      pcl_conversions
      livox_ros_driver2
  )

  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(reloc_manual_xyzquat src/manual/reloc_manual_xyzquat.cpp)
  target_link_libraries(reloc_manual_xyzquat
    PRIVATE
      ${catkin_LIBRARIES}
  )

  add_executable(reloc_manual_xyzyaw src/manual/reloc_manual_xyzyaw.cpp)
  target_link_libraries(reloc_manual_xyzyaw
    PRIVATE
      ${catkin_LIBRARIES}
  )

  add_executable(reloc_livox src/reloc_livox.cpp src/relocalign.cpp)
  target_include_directories(reloc_livox
    PRIVATE
      include
      ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(reloc_livox
    PRIVATE
      ${catkin_LIBRARIES}
      fast_gicp
      yaml-cpp
      ${PCL_LIBRARIES}
  )

  add_executable(demo_reloc src/demo_reloc.cpp src/relocalign.cpp)
  add_dependencies(demo_reloc fast_gicp)
  target_link_libraries(demo_reloc
    PRIVATE
      ${catkin_LIBRARIES}
      fast_gicp
      yaml-cpp
      ${PCL_LIBRARIES}
  )

  add_executable(cloud_collect src/cloud_collect.cpp)
  target_include_directories(cloud_collect
    PRIVATE
      include
      ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(cloud_collect
    PRIVATE
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
  )
endif()
