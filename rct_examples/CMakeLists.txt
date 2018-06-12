cmake_minimum_required(VERSION 2.8.3)
project(rct_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  rct_image_tools
  rct_optimizations
  roscpp

  # TODO: Export these to a different package dedicated to 'ros tools'
  tf2_ros
  cv_bridge
  image_transport
  eigen_conversions
)

find_package(OpenCV REQUIRED)

# This package only provides examples. It does not export any tools.
catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_extrinsic_example
  src/camera_on_wrist_extrinsic.cpp
  src/${PROJECT_NAME}/data_set.cpp)

set_target_properties(${PROJECT_NAME}_extrinsic_example PROPERTIES OUTPUT_NAME cam_on_wrist_extrinsic_example PREFIX "")

add_dependencies(${PROJECT_NAME}_extrinsic_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_extrinsic_example
 ${catkin_LIBRARIES}
 yaml-cpp
 ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_static_camera
  src/static_camera_extrinsic.cpp
  src/${PROJECT_NAME}/data_set.cpp)

set_target_properties(${PROJECT_NAME}_static_camera PROPERTIES OUTPUT_NAME static_camera_cal_example PREFIX "")

add_dependencies(${PROJECT_NAME}_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_static_camera
 ${catkin_LIBRARIES}
 yaml-cpp
 ${OpenCV_LIBRARIES}
)

#Intrinsic test program
add_executable(${PROJECT_NAME}_intr
  src/intrinsic_calibration.cpp
  src/${PROJECT_NAME}/data_set.cpp)

set_target_properties(${PROJECT_NAME}_intr PROPERTIES OUTPUT_NAME intr_camera_cal_example PREFIX "")

add_dependencies(${PROJECT_NAME}_intr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_intr
 ${catkin_LIBRARIES}
 yaml-cpp
 ${OpenCV_LIBRARIES}
)


add_executable(${PROJECT_NAME}_cmd
  src/command_line_cal.cpp
  src/${PROJECT_NAME}/data_set.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME cmd_cal PREFIX "")

add_dependencies(${PROJECT_NAME}_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_cmd
 ${catkin_LIBRARIES}
 yaml-cpp
 ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_pnp
  src/solve_pnp.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME pnp_example PREFIX "")

add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pnp
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
