cmake_minimum_required(VERSION 3.5)
project(publisher)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV
find_package(OpenCV 3.3.1 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

# popl
set(popl_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd/popl/include)
include_directories(${popl_INCLUDE_DIR})

# util
set(util_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/publisher)
include_directories(${util_INCLUDE_DIR})

set(EXECUTABLE_TARGETS "")
add_executable(kitti_images src/kitti_images.cpp ${util_INCLUDE_DIR}/kitti_util.cc)
list(APPEND EXECUTABLE_TARGETS kitti_images)
add_executable(openvslam_video src/openvslam_video.cc)
list(APPEND EXECUTABLE_TARGETS openvslam_video)
add_executable(seeds_images src/seeds_images.cc ${util_INCLUDE_DIR}/seeds_util.cc)
list(APPEND EXECUTABLE_TARGETS seeds_images)
add_executable(euroc_images src/euroc_images.cc ${util_INCLUDE_DIR}/euroc_util.cc)
list(APPEND EXECUTABLE_TARGETS euroc_images)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    target_include_directories(${EXECUTABLE_TARGET} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${EXECUTABLE_TARGET}
      PRIVATE ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})  

    ament_target_dependencies(${EXECUTABLE_TARGET}
      PUBLIC rclcpp cv_bridge image_transport rcutils)

    install(TARGETS ${EXECUTABLE_TARGET}
      EXPORT export_${PROJECT_NAME}
      DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
