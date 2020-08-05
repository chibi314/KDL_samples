cmake_minimum_required(VERSION 3.0.2)
project(KDL_samples)

find_package(catkin REQUIRED COMPONENTS
)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
)

include_directories(
  # include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(sample_ChainFkSolverPos_recursive src/sample_ChainFkSolverPos_recursive.cpp)
target_link_libraries(sample_ChainFkSolverPos_recursive ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(sample_ChainIkSolverPos_NR src/sample_ChainIkSolverPos_NR.cpp)
target_link_libraries(sample_ChainIkSolverPos_NR ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(sample_ChainIkSolverPos_LMA src/sample_ChainIkSolverPos_LMA.cpp)
target_link_libraries(sample_ChainIkSolverPos_LMA ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
