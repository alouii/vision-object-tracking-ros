cmake_minimum_required(VERSION 3.0.2)
project(vision_object_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
  message_generation
  rostest
)

find_package(OpenCV REQUIRED)
find_package(dynamic_reconfigure REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_message_files(
  FILES
  ObjectDetection.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

# rostest
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_detection.test)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Vision.cfg
)

add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(vision_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(vision_node ${PROJECT_NAME}_gencpp)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_node ${PROJECT_NAME}_gencpp)

# Install targets and launch files
install(TARGETS vision_node controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/spawn_and_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
