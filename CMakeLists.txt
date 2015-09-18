# Copyright © 2015 CentraleSupélec
# 
# This file is part of Hand Control.
# 
# Hand Control is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# Hand Control is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with Hand Control.  If not, see <http://www.gnu.org/licenses/>.

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
