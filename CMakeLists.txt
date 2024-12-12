cmake_minimum_required(VERSION 3.0.2)
project(mada)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  move_base_msgs
  actionlib
  roscpp
  rospy
  tf
  laser_geometry

  tf2_ros
  navfn
  base_local_planner
  costmap_2d
)

catkin_package(
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Las librerías que se usan
set(mada_util src/mada_util.cpp src/path_follower.cpp src/projPoints.cpp)
add_library(mada_util ${mada_util})

set(mada_utilities src/util/funciones.cpp src/util/useless.cpp src/util/fmm_2.cpp src/util/generic_fmm.cpp src/util/bresenham.cpp src/util/dynamic.cpp src/util/geometry.cpp   src/util/eq_segm.cpp src/util/grad_segm_def.cpp src/util/it_grad_segm.cpp    src/util/basic_hungarian.cpp src/util/hungarian.cpp src/util/nearest_neighbor.cpp  src/util/cluster.cpp)
add_library(mada_utilities ${mada_utilities})


# Ejecutables
add_executable(mada_obstacles src/obstacles.cpp)
target_link_libraries(mada_obstacles
  LINK_PUBLIC mada_util
  LINK_PUBLIC mada_utilities
  ${catkin_LIBRARIES}
)

add_executable(mada_pf src/mada_pf.cpp)
target_link_libraries(mada_pf
  LINK_PUBLIC mada_util
  LINK_PUBLIC mada_utilities
  ${catkin_LIBRARIES}
)

add_executable(madp_pf src/madp_pf.cpp)
target_link_libraries(madp_pf
  LINK_PUBLIC mada_util
  LINK_PUBLIC mada_utilities
  ${catkin_LIBRARIES}
)

add_executable(greedy_pf src/greedy_pf.cpp)
target_link_libraries(greedy_pf
  LINK_PUBLIC mada_util
  LINK_PUBLIC mada_utilities
  ${catkin_LIBRARIES}
)


