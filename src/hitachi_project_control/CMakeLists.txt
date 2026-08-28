cmake_minimum_required(VERSION 3.8)
project(hitachi_project_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(line_follower src/line_follower.cpp)
ament_target_dependencies(line_follower rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs )

add_executable(web_cam_pub src/web_cam_pub.cpp)
ament_target_dependencies(web_cam_pub rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(aruco_read src/aruco_read.cpp)
ament_target_dependencies(aruco_read rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs tf2 tf2_ros)

add_executable(qr_localisation src/qr_localisation.cpp)
ament_target_dependencies(qr_localisation rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(aruco_vs_odom src/aruco_vs_odom.cpp)
ament_target_dependencies(aruco_vs_odom rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
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

install(TARGETS
  line_follower
  web_cam_pub
  aruco_read
  qr_localisation
  aruco_vs_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
