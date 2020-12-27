cmake_minimum_required(VERSION 3.5)
project(i2c_pwm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  set(CMAKE_CXX_FLAGS "-std=gnu++14 ${CMAKE_CXX_FLAGS}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(i2c_pwm SHARED
  src/MinimalPublisher.cpp
  src/MinimalSubscriber.cpp
  src/Pca9685.cpp
  src/Pca9685Impl.cpp
  src/Servo.cpp
  src/ServoManager.cpp)

ament_export_targets(export_i2c_pwm HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS i2c_pwm
  EXPORT export_i2c_pwm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

target_include_directories(i2c_pwm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  # Include linting tests
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/line_length")

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_cppcheck
  )

  ament_lint_auto_find_test_dependencies()

  include_directories(src)

  ament_add_gtest(i2c_pwm_test
    test/ServoTest.cpp)

  ament_target_dependencies(i2c_pwm_test
    rclcpp)

  target_link_libraries(i2c_pwm_test
    i2c_pwm)

endif()

ament_package()
