cmake_minimum_required(VERSION 2.8.3)
project(rct_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  rct_image_tools
  rct_optimizations
  rct_ros_tools
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

# Executable for demonstrating extrinsic, camera-on-wrist functionality
add_executable(${PROJECT_NAME}_moving_camera src/camera_on_wrist_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_moving_camera PROPERTIES OUTPUT_NAME moving_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_moving_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_moving_camera
 ${catkin_LIBRARIES}
)

# Executable for demonstrating extrinsic cal of static camera, moving target functionality
add_executable(${PROJECT_NAME}_static_camera src/static_camera_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_static_camera PROPERTIES OUTPUT_NAME static_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_static_camera
 ${catkin_LIBRARIES}
)

# Executable for demonstrating extrinsic cal of multiple static camera, moving target functionality
add_executable(${PROJECT_NAME}_multi_static_camera src/multi_static_camera_extrinsic.cpp)

set_target_properties(${PROJECT_NAME}_multi_static_camera PROPERTIES OUTPUT_NAME multi_static_cam_extr_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_multi_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_multi_static_camera
 ${catkin_LIBRARIES}
)

#Executable for demonstrating intrinsic calibration of a camera
add_executable(${PROJECT_NAME}_intr
  src/intrinsic_calibration.cpp)

set_target_properties(${PROJECT_NAME}_intr PROPERTIES OUTPUT_NAME intr_camera_cal_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_intr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_intr
 ${catkin_LIBRARIES}
)

# Executable demonstrating solving for the pose of a target given camera properties
add_executable(${PROJECT_NAME}_pnp src/solve_pnp.cpp)

set_target_properties(${PROJECT_NAME}_pnp PROPERTIES OUTPUT_NAME solve_pnp_ex PREFIX "")

add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pnp
 ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(${PROJECT_NAME}_cam_on_wrist_test
#    test/cam_on_wrist.test # The launch file
#    test/camera_on_wrist_extrinsic_test.cpp # The source file(s)
#  )
#  target_link_libraries(${PROJECT_NAME}_cam_on_wrist_test ${catkin_LIBRARIES})
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
