cmake_minimum_required(VERSION 3.18)
project(camera_ros)
set(CMAKE_CXX_STANDARD 20)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

# Find dependencies
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera>=0.1)

# Composable ROS2 node
add_library(camera_component SHARED src/CameraNode.cpp   src/WbCameraHandler.cpp)
rclcpp_components_register_node(camera_component
  PLUGIN "camera::MinimalCameraNode"
  EXECUTABLE "camera_node")

set(PACKAGE_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "camera_info_manager"
)

ament_target_dependencies(camera_component PUBLIC ${PACKAGE_DEPENDENCIES})
target_include_directories(camera_component PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(camera_component PUBLIC ${libcamera_LINK_LIBRARIES})

# Install targets
ament_export_targets(camera_componentTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

install(TARGETS camera_component
  EXPORT camera_componentTargets
  DESTINATION lib
)

# Install additional directories if needed
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
