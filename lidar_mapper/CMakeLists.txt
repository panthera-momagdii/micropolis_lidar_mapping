cmake_minimum_required(VERSION 3.16)
project(lidar_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)  # parallelize the per-point registration loop

add_library(lm_core
  src/point_cloud.cpp
  src/bag_stream.cpp
  src/tum_io.cpp
  src/pcd_io.cpp
  src/deskew.cpp
  src/voxel_map.cpp
  src/registration.cpp
  src/odometry.cpp
)
target_include_directories(lm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lm_core
  rclcpp rosbag2_cpp rosbag2_storage sensor_msgs nav_msgs)
target_link_libraries(lm_core Eigen3::Eigen OpenMP::OpenMP_CXX)

add_executable(inspect_bags src/tools/inspect_bags.cpp)
target_link_libraries(inspect_bags lm_core)

add_executable(run_odometry src/tools/run_odometry.cpp)
target_link_libraries(run_odometry lm_core yaml-cpp)

add_executable(export_gt src/tools/export_gt.cpp)
target_link_libraries(export_gt lm_core)

install(TARGETS lm_core
  EXPORT export_lm_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS inspect_bags run_odometry export_gt
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# export lm_core so gnss_fusion can link it + consume headers via find_package(lidar_mapper)
ament_export_include_directories(include)
ament_export_targets(export_lm_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_storage sensor_msgs nav_msgs Eigen3 OpenMP)

ament_package()
