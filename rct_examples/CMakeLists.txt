cmake_minimum_required(VERSION 2.8.3)
project(rct_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  rct_image_tools
  rct_optimizations
  roscpp
)

find_package(OpenCV REQUIRED)

# This package only provides examples. It does not export any tools.
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_extrinsic_example src/camera_on_wrist_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_extrinsic_example PROPERTIES OUTPUT_NAME cam_on_wrist_extrinsic_example PREFIX "")

add_dependencies(${PROJECT_NAME}_extrinsic_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_extrinsic_example
 ${catkin_LIBRARIES}
 yaml-cpp
 ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_extrinsic_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
