cmake_minimum_required(VERSION 3.5.0)
project(rct_ros_tools)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(rct_optimizations REQUIRED)
find_package(rct_image_tools REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
if(NOT TARGET Eigen3::Eigen)
    find_package(Threads REQUIRED)
    add_library(Eigen3::Eigen IMPORTED INTERFACE)
    set_target_properties (Eigen3::Eigen PROPERTIES
      EXPORT_NAME Eigen
      INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS}
      INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
      )
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp

  # TODO: Export these to a different package dedicated to 'ros tools'
  tf2_ros
  cv_bridge
  image_transport
  eigen_conversions
)

# This package only provides examples. It does not export any tools.
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_data_loader
    yaml-cpp
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Create a library for loading and saving "data sets": directory based
# archives of poses and images that can be reloaded for testing and
# development purposes.
add_library(${PROJECT_NAME}_data_loader
  src/data_loader/data_set.cpp
  src/data_loader/parameter_loaders.cpp)

add_dependencies(${PROJECT_NAME}_data_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_data_loader
 yaml-cpp
 ${OpenCV_LIBRARIES}
 rct::rct_optimizations
 rct::rct_image_tools
)

# Executable for collecting data sets via subscribers and triggered with services
# See readme (TODO: Write a readme)
add_executable(${PROJECT_NAME}_cmd src/command_line_cal.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME command_line_data_collection PREFIX "")

add_dependencies(${PROJECT_NAME}_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_cmd
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_loader
 rct::rct_image_tools
)

#############
## Testing ##
#############

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_data_loader
    ${PROJECT_NAME}_cmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
