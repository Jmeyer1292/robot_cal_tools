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
catkin_package()

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
add_library(${PROJECT_NAME}_data_set src/${PROJECT_NAME}/data_set.cpp)

add_dependencies(${PROJECT_NAME}_data_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_data_set
 yaml-cpp
 ${OpenCV_LIBRARIES}
)

# Executable for demonstrating extrinsic, camera-on-wrist functionality
add_executable(${PROJECT_NAME}_moving_camera src/camera_on_wrist_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_moving_camera PROPERTIES OUTPUT_NAME moving_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_moving_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_moving_camera
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_set
)

# Executable for demonstrating extrinsic cal of static camera, moving target functionality
add_executable(${PROJECT_NAME}_static_camera src/static_camera_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_static_camera PROPERTIES OUTPUT_NAME static_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_static_camera
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_set
)

#Executable for demonstrating intrinsic calibration of a camera
add_executable(${PROJECT_NAME}_intr
  src/intrinsic_calibration.cpp)

set_target_properties(${PROJECT_NAME}_intr PROPERTIES OUTPUT_NAME intr_camera_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_intr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_intr
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_set

)

# Executable demonstrating solving for the pose of a target given camera properties
add_executable(${PROJECT_NAME}_pnp src/solve_pnp.cpp)

set_target_properties(${PROJECT_NAME}_pnp PROPERTIES OUTPUT_NAME solve_pnp_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pnp
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_set
)

# Executable for collecting data sets via subscribers and triggered with services
# See readme (TODO: Write a readme)
add_executable(${PROJECT_NAME}_cmd src/command_line_cal.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME command_line_data_collection PREFIX "")

add_dependencies(${PROJECT_NAME}_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_cmd
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_set
)


#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_moving_camera
    ${PROJECT_NAME}_static_camera
    ${PROJECT_NAME}_intr
    ${PROJECT_NAME}_pnp
    ${PROJECT_NAME}_cmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
