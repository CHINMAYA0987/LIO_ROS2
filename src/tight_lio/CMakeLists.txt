cmake_minimum_required(VERSION 3.14)
project(tight_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -march=native)

# ── ROS2 dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# ── Third-party ────────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters io)

# ── ikd-Tree (git submodule) ───────────────────────────────────────────────────
set(IKD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikd-Tree/ikd-Tree)
if(NOT EXISTS ${IKD_DIR}/ikd_Tree.cpp)
  message(FATAL_ERROR
    "ikd-Tree submodule not found at ${IKD_DIR}.\n"
    "Run:  git submodule update --init --recursive")
endif()

# ── Include dirs ──────────────────────────────────────────────────────────────
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${IKD_DIR}
)

# ── Library sources ───────────────────────────────────────────────────────────
set(LIB_SOURCES
  src/imu_integrator.cpp
  src/point_deskewer.cpp
  src/icp_matcher.cpp
  src/iekf_estimator.cpp
  ${IKD_DIR}/ikd_Tree.cpp
)

add_library(tight_lio_lib ${LIB_SOURCES})
target_link_libraries(tight_lio_lib
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(tight_lio_lib
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_eigen
  pcl_conversions pcl_ros
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(tight_lio_node src/tight_lio_node.cpp)
target_link_libraries(tight_lio_node tight_lio_lib)
ament_target_dependencies(tight_lio_node
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_eigen
  pcl_conversions pcl_ros
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS tight_lio_node tight_lio_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()