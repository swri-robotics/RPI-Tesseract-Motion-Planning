cmake_minimum_required(VERSION 3.15)
project(robot_motion_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tesseract_msgs
  tesseract_monitoring
  tesseract_planning_server
  tesseract_rosutils
  tf2_ros
  roscpp
  rpi_abb_irb6640_180_255_support
  message_generation
)

find_package(Boost REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(yaml-cpp REQUIRED)

# Custom interface generation
add_service_files(FILES
  PlanTrajectory.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  tesseract_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    planner_utils
  CATKIN_DEPENDS
    geometry_msgs
    tesseract_msgs
    tesseract_monitoring
    tesseract_planning_server
    tesseract_rosutils
    roscpp
    tf2_ros
    std_srvs
    message_runtime
   DEPENDS
    tesseract_environment
    tesseract_scene_graph
    tesseract_command_language
    tesseract_process_managers
    tesseract_kinematics
)

# add_library(planner_utils src/planner_utils.cpp)
# target_include_directories(planner_utils PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include>"
# ${YAML_CPP_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(planner_utils
# ${catkin_LIBRARIES}
# ${YAML_CPP_LIBRARIES}
# tesseract::tesseract_command_language
# tesseract::tesseract_process_managers
# )

add_executable(planner_node
  src/planner_node.cpp
  src/planner_utils.cpp
)
target_include_directories(planner_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(planner_node
  # planner_utils
  ${catkin_LIBRARIES}
  Boost::headers
  tesseract::tesseract_command_language
  tesseract::tesseract_process_managers
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS
  # planner_utils
    planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
