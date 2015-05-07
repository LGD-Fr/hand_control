cmake_minimum_required(VERSION 2.8.3)
project(hand_control)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_msgs
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  ardrone_autonomy
)
find_package(
  PkgConfig REQUIRED
)

pkg_check_modules ( ncursesw REQUIRED ncursesw)

add_message_files(
  FILES
  Plan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ncursesw_INCLUDE_DIRS}
)

add_executable(filtre src/filtre.cpp)
target_link_libraries(filtre ${catkin_LIBRARIES})

add_executable(random_pcl_publisher src/random_pcl_publisher.cpp)
target_link_libraries(random_pcl_publisher ${catkin_LIBRARIES})

add_executable(pcl_displayer src/pcl_displayer.cpp)
target_link_libraries(pcl_displayer ${catkin_LIBRARIES})

add_library(display src/display.cpp)

add_executable(keyboard_cmd src/keyboard_cmd.cpp)
target_link_libraries(keyboard_cmd display ${catkin_LIBRARIES} ${ncursesw_LIBRARIES})

add_executable(filtreRGB src/filtreRGB.cpp)
target_link_libraries(filtreRGB ${catkin_LIBRARIES})

#add_executable(filtreHue src/filtreHue.cpp)
#target_link_libraries(filtreHue ${catkin_LIBRARIES})

add_executable(normal_estimator src/normal_estimator.cpp)
target_link_libraries(normal_estimator ${catkin_LIBRARIES})
add_dependencies(normal_estimator hand_control_generate_messages_cpp)

add_executable(commande src/commande.cpp)
target_link_libraries(commande ${catkin_LIBRARIES})
