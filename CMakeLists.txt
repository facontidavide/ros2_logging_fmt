cmake_minimum_required(VERSION 3.5)
project(ros2_logging_fmt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)


add_executable(test_logging src/test_logging.cpp)

target_include_directories(test_logging
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(test_logging rclcpp fmt)

ament_export_dependencies(rclcpp fmt)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS test_logging
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
