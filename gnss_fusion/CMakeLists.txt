cmake_minimum_required(VERSION 3.16)
project(gnss_fusion)

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
find_package(lidar_mapper REQUIRED)  # provides the exported lidar_mapper::lm_core library + headers

add_library(gf_core src/pose_graph.cpp)
target_include_directories(gf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gf_core lidar_mapper::lm_core Eigen3::Eigen)  # se3.hpp comes from lidar_mapper::lm_core's headers

add_executable(run_fusion src/tools/run_fusion.cpp)
ament_target_dependencies(run_fusion rclcpp rosbag2_cpp rosbag2_storage nav_msgs)
target_link_libraries(run_fusion gf_core lidar_mapper::lm_core yaml-cpp)

# regen_map reuses Variant A's locked cloud pipeline (deskew + voxel output) by linking lidar_mapper::lm_core —
# the same compiled objects Variant A uses, not a recompile. only the per-scan pose changes.
add_executable(regen_map src/tools/regen_map.cpp)
ament_target_dependencies(regen_map rclcpp rosbag2_cpp rosbag2_storage sensor_msgs nav_msgs)
target_link_libraries(regen_map lidar_mapper::lm_core yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS run_fusion regen_map RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
