cmake_minimum_required(VERSION 3.8)
project(pcl_type_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pcl_type_adapter.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_type_adapter test/test_type_adapter.cpp)
  target_link_libraries(test_type_adapter ${PROJECT_NAME})
endif()

ament_auto_package()
