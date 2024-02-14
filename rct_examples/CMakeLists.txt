cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(rct_optimizations REQUIRED)
find_package(rct_image_tools REQUIRED)
find_package(rct_common REQUIRED)

find_package(catkin REQUIRED COMPONENTS rct_ros_tools roslib)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# This package only provides examples. It does not export any tools.
catkin_package()

# ######################################################################################################################
# Build ##
# ######################################################################################################################

include_directories(${catkin_INCLUDE_DIRS})

# ######################################################################################################################
# SIMPLE Examples ##
# ######################################################################################################################
# Executable for extrinsic camera on wrist calibration
add_executable(${PROJECT_NAME}_wrist_example src/examples/camera_on_wrist.cpp)

set_target_properties(${PROJECT_NAME}_wrist_example PROPERTIES OUTPUT_NAME example_camera_on_wrist PREFIX "")

add_dependencies(${PROJECT_NAME}_wrist_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_wrist_example
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for kinematic calibration
add_executable(${PROJECT_NAME}_kinematic_calibration src/examples/kinematic_calibration.cpp)

set_target_properties(${PROJECT_NAME}_kinematic_calibration PROPERTIES OUTPUT_NAME kinematic_calibration PREFIX "")

add_dependencies(${PROJECT_NAME}_kinematic_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_kinematic_calibration
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# ######################################################################################################################
# Offline Calibration Tools ##
# ######################################################################################################################
# Executable for demonstrating extrinsic, camera-on-wrist functionality
add_executable(${PROJECT_NAME}_moving_camera src/tools/camera_on_wrist_extrinsic.cpp)
set_target_properties(${PROJECT_NAME}_moving_camera PROPERTIES OUTPUT_NAME moving_cam_extr_cal_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_moving_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_moving_camera
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for demonstrating extrinsic cal of static camera, moving target functionality
add_executable(${PROJECT_NAME}_static_camera src/tools/static_camera_extrinsic.cpp)
set_target_properties(${PROJECT_NAME}_static_camera PROPERTIES OUTPUT_NAME static_cam_extr_cal_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_static_camera
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for demonstrating extrinsic cal of multiple static camera, moving target functionality
add_executable(${PROJECT_NAME}_multi_static_camera src/tools/multi_static_camera_extrinsic.cpp)
set_target_properties(${PROJECT_NAME}_multi_static_camera PROPERTIES OUTPUT_NAME multi_static_cam_extr_cal_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_multi_static_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_multi_static_camera
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for demonstrating extrinsic cal of multiple static camera multi step, moving target functionality First it
# calibrates the cameras to each other then it calibrates the set of camera where there relationship between cameras is
# fixed but as a whole they can be transformed along with wrist calibration.
add_executable(${PROJECT_NAME}_multi_static_camera_multi_step src/tools/multi_static_camera_multi_step_extrinsic.cpp)
set_target_properties(${PROJECT_NAME}_multi_static_camera_multi_step
                      PROPERTIES OUTPUT_NAME multi_static_cam_multi_step_extr_cal_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_multi_static_camera_multi_step ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_multi_static_camera_multi_step
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for demonstrating intrinsic calibration of a camera
add_executable(${PROJECT_NAME}_intr src/tools/intrinsic_calibration.cpp)
set_target_properties(${PROJECT_NAME}_intr PROPERTIES OUTPUT_NAME intr_camera_cal_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_intr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_intr
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable demonstrating solving for the pose of a target given camera properties
add_executable(${PROJECT_NAME}_pnp src/tools/solve_pnp.cpp)
set_target_properties(${PROJECT_NAME}_pnp PROPERTIES OUTPUT_NAME solve_pnp_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_pnp
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable demonstrating solving for the pose of a target given multiple camera properties
add_executable(${PROJECT_NAME}_multi_camera_pnp src/tools/solve_multi_camera_pnp.cpp)
set_target_properties(${PROJECT_NAME}_multi_camera_pnp PROPERTIES OUTPUT_NAME solve_multi_camera_pnp_ex PREFIX "")
add_dependencies(${PROJECT_NAME}_multi_camera_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_multi_camera_pnp
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable demonstrating camera intrinsic calibration validation
add_executable(${PROJECT_NAME}_camera_intrinsic_calibration_validation
               src/tools/camera_intrinsic_calibration_validation.cpp)
set_target_properties(${PROJECT_NAME}_camera_intrinsic_calibration_validation
                      PROPERTIES OUTPUT_NAME camera_intrinsic_calibration_validation PREFIX "")
add_dependencies(${PROJECT_NAME}_camera_intrinsic_calibration_validation ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_camera_intrinsic_calibration_validation
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# Executable for testing camera noise
add_executable(${PROJECT_NAME}_noise_qualification_2d src/tools/noise_qualification_2d.cpp)
set_target_properties(${PROJECT_NAME}_noise_qualification_2d PROPERTIES OUTPUT_NAME noise_qualification_2d PREFIX "")
add_dependencies(${PROJECT_NAME}_noise_qualification_2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_noise_qualification_2d
  ${catkin_LIBRARIES}
  rct::rct_optimizations
  rct::rct_image_tools
  rct::rct_common)

# ######################################################################################################################
# Testing ##
# ######################################################################################################################
if(CATKIN_ENABLE_TESTING AND RCT_BUILD_TESTS)
  find_package(GTest REQUIRED)
  # Tests extrinsic wrist calibration example
  catkin_add_gtest(${PROJECT_NAME}_wrist_test src/examples/camera_on_wrist.cpp)
  target_compile_definitions(${PROJECT_NAME}_wrist_test PRIVATE -DRCT_ENABLE_TESTING)
  target_link_libraries(
    ${PROJECT_NAME}_wrist_test
    ${catkin_LIBRARIES}
    rct::rct_optimizations
    rct::rct_image_tools
    rct::rct_common
    GTest::GTest
    GTest::Main)
endif()

# ######################################################################################################################
# Install ##
# ######################################################################################################################
install(
  TARGETS ${PROJECT_NAME}_moving_camera
          ${PROJECT_NAME}_static_camera
          ${PROJECT_NAME}_multi_static_camera
          ${PROJECT_NAME}_intr
          ${PROJECT_NAME}_pnp
          ${PROJECT_NAME}_camera_intrinsic_calibration_validation
          ${PROJECT_NAME}_noise_qualification_2d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
