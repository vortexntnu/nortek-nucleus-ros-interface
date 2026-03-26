cmake_minimum_required(VERSION 3.8)
project(nortek_nucleus_ros_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nortek_nucleus_driver CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/nortek_nucleus_ros_interface.cpp
)

target_link_libraries(
    ${LIB_NAME}
    PRIVATE
        nortek_nucleus_driver::nortek_nucleus_driver
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "NortekNucleusRosInterface"
    EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
