cmake_minimum_required(VERSION 3.0.2)
project(relocalign)

set(CMAKE_BUILD_TYPE "Release")
set(RELOCALIGN_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
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
  # ${EIGEN3_INCLUDE_DIR} # Do not add these, please use target target_include_directories() !
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

add_executable(reloc_icp src/reloc_icp.cpp src/relocalign.cpp)
target_link_libraries(reloc_icp 
  PRIVATE
    ${catkin_LIBRARIES}
    fast_gicp
    yaml-cpp
  )

add_executable(demo_icp src/demo_icp.cpp src/relocalign.cpp)
add_dependencies(demo_icp fast_gicp)
target_link_libraries(demo_icp 
  PRIVATE
    fast_gicp
    yaml-cpp
  )


