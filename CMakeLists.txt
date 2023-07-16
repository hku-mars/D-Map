cmake_minimum_required(VERSION 2.8.3)
project(dmap)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3 -fopenmp -DCMAKE_BUILD_TYPE=Debug")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(fmt REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS cmake_modules geometry_msgs nav_msgs pcl_ros roscpp roslib sensor_msgs
  DEPENDS EIGEN PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

message(STATUS "Super Ray Grid Map3D include dir: ${GRIDMAP3D_LIBRARIES}")

# # Declare a C++ library
add_library(${PROJECT_NAME}_lib
  include/D-Map/config.cpp
  include/D-Map/D-Map.cpp
  include/D-Map/octree_map.cpp
  include/D-Map/GridMap.cpp
)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
# include/DecrementalMap/DecrementalMap.cpp
add_executable(example src/DMap_example.cpp)
target_link_libraries(example
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_lib
  fmt::fmt
)