cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🚀 Required ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 🚀 Required for service generation
find_package(rosidl_default_generators REQUIRED)

# 🚀 Generate ROS2 interfaces (make sure the path is correct)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  DEPENDENCIES std_msgs nav_msgs visualization_msgs
)

# 🚀 Ensure dependencies on generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# 🚀 Make sure executables are properly linked
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
ament_target_dependencies(trajectory_publisher_saver
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
ament_target_dependencies(trajectory_reader_publisher
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 🚀 Link the generated service types
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_publisher_saver ${cpp_typesupport_target})
target_link_libraries(trajectory_reader_publisher ${cpp_typesupport_target})

# 🚀 Install the executables
install(TARGETS
  trajectory_publisher_saver
  trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 🚀 Install the service files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_package()

