cmake_minimum_required(VERSION 3.0.2)
project(robot_motion_planning)

find_package(catkin REQUIRED COMPONENTS
  tesseract_monitoring
  tesseract_planning_server
  tesseract_rosutils
  tf2_ros
  geometry_msgs
  roscpp
  rpi_abb_irb6640_180_255_support
  std_msgs
)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  ## LIBRARIES
  CATKIN_DEPENDS
    tesseract_monitoring
    tesseract_planning_server
    tesseract_rosutils
    roscpp
    tf2_ros
    std_srvs
   DEPENDS
    tesseract_environment
    tesseract_scene_graph
    tesseract_command_language
    tesseract_process_managers
    tesseract_kinematics
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  tesseract::tesseract_command_language
  tesseract::tesseract_process_managers
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(
  TARGETS
    planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package()
