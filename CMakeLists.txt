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
               dynamic_reconfigure
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


  include_directories(
                      ${catkin_INCLUDE_DIRS}
                      ${ncursesw_INCLUDE_DIRS}
                     )

  add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})

add_library(display src/display.cpp)

  add_executable(keyboard_cmd src/keyboard_cmd.cpp)
target_link_libraries(keyboard_cmd display ${catkin_LIBRARIES} ${ncursesw_LIBRARIES})

  add_executable(estimator src/estimator.cpp)
target_link_libraries(estimator ${catkin_LIBRARIES})
add_dependencies(estimator hand_control_generate_messages_cpp)

  add_executable(commander src/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})
add_dependencies(commander hand_control_generate_messages_cpp)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Filter.cfg
  cfg/Commander.cfg
  cfg/Estimator.cfg
  )

add_dependencies(filter ${PROJECT_NAME}_gencfg)
add_dependencies(commander ${PROJECT_NAME}_gencfg)
add_dependencies(estimator ${PROJECT_NAME}_gencfg)

  catkin_package(
                 CATKIN_DEPENDS message_runtime
                )
