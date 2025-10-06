cmake_minimum_required(VERSION 3.0.2)
project(relocalign)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(RELOCALIGN_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  # message_runtime
  # pcl_conversions
)


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reloc_manual
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR} # Do not add these, please use target_include_directories() !
  # ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src/fast_gicp)

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

if(BUILD_VGICP_CUDA)
  add_definitions(-DUSE_VGICP_CUDA)
endif()

add_executable(reloc_livox src/reloc_livox.cpp src/relocalign.cpp)
target_include_directories(reloc_livox
  PRIVATE
    ${CATKIN_DEVEL_PREFIX}/include
    ${PCL_INCLUDE_DIRS}
  )
target_link_libraries(reloc_livox 
  PRIVATE
    ${catkin_LIBRARIES}
    fast_gicp
    yaml-cpp
    ${CATKIN_DEVEL_PREFIX}/lib
    ${PCL_LIBRARIES}
  )

add_executable(demo_reloc src/demo_reloc.cpp src/relocalign.cpp)
add_dependencies(demo_reloc fast_gicp)
target_link_libraries(demo_reloc 
  PRIVATE
    ${catkin_LIBRARIES}
    fast_gicp
    yaml-cpp
  )

add_executable(cloud_collect src/cloud_collect.cpp)
target_include_directories(cloud_collect
  PRIVATE
    ${PCL_INCLUDE_DIRS}
  )
target_link_libraries(cloud_collect 
  PRIVATE
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )


