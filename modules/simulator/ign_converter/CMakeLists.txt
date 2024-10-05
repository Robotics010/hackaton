cmake_minimum_required(VERSION 3.8)
project(ign_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_library(converter SHARED
  src/ign_converter.cpp
)

include_directories(
  include
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)
ament_target_dependencies(converter
  rclcpp
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  converter
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
