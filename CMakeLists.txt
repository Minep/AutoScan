cmake_minimum_required(VERSION 3.8)
project(auto_scanner)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ORB_SLAM3_LOCATION})
  set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ucl-cs/y4/fyp/ORB_SLAM3")
else()
  set(ORB_SLAM3_ROOT_DIR $ENV{ORB_SLAM3_LOCATION})
endif()

if(NOT DEFINED ENV{Open3D_LOCATION})
  set(O3D_DIR "$ENV{HOME}/opt/open3d")
else()
  set(O3D_DIR $ENV{Open3D_LOCATION})
endif()

if(NOT DEFINED ENV{OCTOMAP_LOCATION})
  set(OCTOMAP_DIR "$ENV{HOME}/opt/octomap")
else()
  set(OCTOMAP_DIR $ENV{OCTOMAP_LOCATION})
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(Octomap REQUIRED)

find_package(rclpy REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${Octomap_INCLUDE_DIRS}
  "install/auto_scanner/include/auto_scanner"
)

find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PosedImage.msg"
    "msg/PosedRGBD.msg"
    "srv/CaptureState.srv"
    "srv/NodeReady.srv"
    DEPENDENCIES sensor_msgs
)

ament_python_install_package(simplerecon)
ament_python_install_package(options)
ament_python_install_package(mesh)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(
  gstreaming 
  src/gstreaming/gstreaming.cpp 
  src/param_helper.cpp 
  src/health_node.cpp
)
ament_target_dependencies(gstreaming rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
target_link_libraries(gstreaming "${cpp_typesupport_target}") 

add_executable(
  orb_slam3 
  src/orb_slam3/orb_slam_ros.cpp 
  src/health_node.cpp
  src/param_helper.cpp
)
ament_target_dependencies(orb_slam3 rclcpp std_msgs geometry_msgs sensor_msgs ORB_SLAM3 OpenCV cv_bridge Pangolin rclcpp_lifecycle)
target_link_libraries(orb_slam3 "${cpp_typesupport_target}") 

add_executable(
  occupancy 
  src/o3d_octomap/occupancy.cpp 
  src/fuser.cpp src/param_helper.cpp 
  src/health_node.cpp
)
ament_target_dependencies(occupancy rclcpp std_msgs geometry_msgs sensor_msgs OpenCV Open3D Octomap cv_bridge)
target_link_libraries(occupancy "${cpp_typesupport_target}")

install(TARGETS
  orb_slam3
  occupancy
  gstreaming
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/extractor.py
  scripts/fuser.py
  scripts/control_panel.py
  scripts/navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY data/ DESTINATION  share/${PROJECT_NAME}/data/)

ament_package()