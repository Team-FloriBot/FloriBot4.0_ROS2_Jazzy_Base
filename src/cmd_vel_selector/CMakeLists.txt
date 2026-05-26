cmake_minimum_required(VERSION 3.8)
project(cmd_vel_selector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Eigener Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectSource.srv"
)

# Node
add_executable(cmd_vel_selector_node
  src/cmd_vel_selector_node.cpp
)

ament_target_dependencies(cmd_vel_selector_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)

# Der Node verwendet den Service, der im selben Package definiert wird.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(cmd_vel_selector_node
  "${cpp_typesupport_target}"
)

# Installation
install(TARGETS
  cmd_vel_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
