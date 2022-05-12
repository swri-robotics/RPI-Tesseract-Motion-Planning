cmake_minimum_required(VERSION 3.0.2)
project(rpi_abb_irb6640_180_255_support)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config meshes)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
