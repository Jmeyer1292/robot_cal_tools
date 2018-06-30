cmake_minimum_required(VERSION 2.8.3)
project(rct_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  rct_image_tools
  rct_optimizations
  rct_ros_tools
  roslib
)

find_package(OpenCV REQUIRED)

# This package only provides examples. It does not export any tools.
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#####################
## SIMPLE Examples ##
#####################
add_executable(${PROJECT_NAME}_wrist_example src/examples/camera_on_wrist.cpp)

set_target_properties(${PROJECT_NAME}_wrist_example PROPERTIES OUTPUT_NAME example_camera_on_wrist PREFIX "")

add_dependencies(${PROJECT_NAME}_wrist_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_wrist_example ${catkin_LIBRARIES})

###############################
## Offline Calibration Tools ##
###############################
# Executable for demonstrating extrinsic, camera-on-wrist functionality
add_executable(${PROJECT_NAME}_moving_camera src/tools/camera_on_wrist_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_moving_camera PROPERTIES OUTPUT_NAME moving_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_moving_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_moving_camera ${catkin_LIBRARIES})

# Executable for demonstrating extrinsic cal of static camera, moving target functionality
add_executable(${PROJECT_NAME}_static_camera src/tools/static_camera_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_static_camera PROPERTIES OUTPUT_NAME static_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_static_camera ${catkin_LIBRARIES})

# Executable for demonstrating extrinsic cal of multiple static camera, moving target functionality
add_executable(${PROJECT_NAME}_multi_static_camera src/tools/multi_static_camera_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_multi_static_camera PROPERTIES OUTPUT_NAME multi_static_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_multi_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_multi_static_camera ${catkin_LIBRARIES})

# Executable for demonstrating extrinsic cal of multiple static camera only, moving target functionality
add_executable(${PROJECT_NAME}_multi_static_camera_only src/tools/multi_static_camera_only_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_multi_static_camera_only PROPERTIES OUTPUT_NAME multi_static_cam_only_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_multi_static_camera_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_multi_static_camera_only ${catkin_LIBRARIES})

#Executable for demonstrating intrinsic calibration of a camera
add_executable(${PROJECT_NAME}_intr src/tools/intrinsic_calibration.cpp)

set_target_properties(${PROJECT_NAME}_intr PROPERTIES OUTPUT_NAME intr_camera_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_intr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_intr ${catkin_LIBRARIES})

# Executable demonstrating solving for the pose of a target given camera properties
add_executable(${PROJECT_NAME}_pnp src/tools/solve_pnp.cpp)

set_target_properties(${PROJECT_NAME}_pnp PROPERTIES OUTPUT_NAME solve_pnp_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pnp ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Tests extrinsic wrist calibration example
  catkin_add_gtest(${PROJECT_NAME}_wrist_test src/examples/camera_on_wrist.cpp)
  target_compile_definitions(${PROJECT_NAME}_wrist_test PRIVATE -DRCT_ENABLE_TESTING)
  target_link_libraries(${PROJECT_NAME}_wrist_test ${catkin_LIBRARIES})
endif()

#############
## Install ##
#############
install(TARGETS
    ${PROJECT_NAME}_moving_camera
    ${PROJECT_NAME}_static_camera
    ${PROJECT_NAME}_multi_static_camera
    ${PROJECT_NAME}_intr
    ${PROJECT_NAME}_pnp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
