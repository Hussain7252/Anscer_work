cmake_minimum_required(VERSION 3.8)
project(trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

# Executable for trajectory publisher and saver
add_executable(traj_pub_saver_node src/traj_pub_saver_node.cpp)
ament_target_dependencies(traj_pub_saver_node
  rclcpp nav_msgs geometry_msgs visualization_msgs
)
rosidl_target_interfaces(traj_pub_saver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS traj_pub_saver_node DESTINATION lib/${PROJECT_NAME})

# Executable for trajectory reader
add_executable(trajectory_reader_node src/trajectory_reader.cpp)
target_include_directories(trajectory_reader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_reader_node
  rclcpp visualization_msgs yaml-cpp
)
target_link_libraries(trajectory_reader_node yaml-cpp)
install(TARGETS trajectory_reader_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install parameter files
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

