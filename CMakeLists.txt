cmake_minimum_required(VERSION 3.8)
project(mobile_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-g) # uncomment to debug

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Markers.msg"
  "msg/TriangulatedMarkers.msg"
  "msg/Benchmark.msg"
  "srv/SetCams.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/calibrate.py
  scripts/setup_cameras.py
  scripts/rigid_body_tracker.py
  scripts/determine_matrices.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers
install(
  DIRECTORY "include/"
  DESTINATION include/${PROJECT_NAME}
)

# Install Cpp executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(find_markers src/find_markers.cpp )
ament_target_dependencies(find_markers rclcpp cv_bridge OpenCV)
target_link_libraries(find_markers "${cpp_typesupport_target}") 

add_executable(triangulate src/triangulate.cpp)
ament_target_dependencies(triangulate message_filters rclcpp cv_bridge OpenCV visualization_msgs geometry_msgs)
target_link_libraries(triangulate "${cpp_typesupport_target}") 

add_executable(publish_feed src/publish_feed.cpp)
ament_target_dependencies(publish_feed rclcpp cv_bridge OpenCV camera_info_manager)


target_include_directories(find_markers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
target_include_directories(publish_feed
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

target_include_directories(triangulate
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

install(TARGETS 
        find_markers
        triangulate
        publish_feed
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
