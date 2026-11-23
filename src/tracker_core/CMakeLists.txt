cmake_minimum_required(VERSION 3.16)
project(tracker_core LANGUAGES CXX)

# C++17, no compiler-specific extensions (portable code).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Treat common mistakes as visible warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# The tracker library (Kalman filter, Hungarian algorithm, track manager)
# will be added here from G6 on, e.g.:
#   add_library(${PROJECT_NAME} src/kalman_filter.cpp ...)
#   target_include_directories(${PROJECT_NAME} PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
#   target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests will be registered here, e.g.:
  #   ament_add_gtest(test_kalman_filter test/test_kalman_filter.cpp)
  #   target_link_libraries(test_kalman_filter ${PROJECT_NAME})
endif()

ament_package()
