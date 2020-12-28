cmake_minimum_required(VERSION 3.5)
project(i2c_pwm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/Pca9685.cpp
  src/Pca9685Impl.cpp
  src/Pca9685Mock.cpp
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_package()
