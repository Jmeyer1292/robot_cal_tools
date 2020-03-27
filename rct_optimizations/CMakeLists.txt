cmake_minimum_required(VERSION 2.8.3)
project(rct_optimizations)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${CERES_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Ceres
)

###########
## Build ##
###########

include_directories(
  include
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  # Utilities
  src/${PROJECT_NAME}/eigen_conversions.cpp
  # Optimizations (monocular camera)
  src/${PROJECT_NAME}/extrinsic_camera_on_wrist.cpp
  src/${PROJECT_NAME}/extrinsic_static_camera.cpp
  # Optimizations (multiple cameras)
  src/${PROJECT_NAME}/extrinsic_multi_static_camera.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_only.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_wrist_only.cpp
  # Optimizations (3D sensor)
  src/${PROJECT_NAME}/extrinsic_3d_camera_on_wrist.cpp
  # Optimizations (Experimental) - Intrinsic
  src/${PROJECT_NAME}/camera_intrinsic.cpp
  src/${PROJECT_NAME}/pnp.cpp
  src/${PROJECT_NAME}/multi_camera_pnp.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Build the test support library
  add_library(${PROJECT_NAME}_test_support
    test/src/observation_creator.cpp
    test/src/utilities.cpp)
  target_link_libraries(${PROJECT_NAME}_test_support ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test_support PUBLIC test/include)

  # The actual tests...
  catkin_add_gtest(${PROJECT_NAME}_conversion_tests test/conversion_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_conversion_tests ${PROJECT_NAME}_test_support)

  catkin_add_gtest(${PROJECT_NAME}_ceres_math_utilities_tests test/ceres_math_utilities_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_ceres_math_utilities_tests ${PROJECT_NAME}_test_support)

  catkin_add_gtest(${PROJECT_NAME}_extrinsic_multi_static_camera_tests test/extrinsic_multi_static_camera_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_extrinsic_multi_static_camera_tests ${PROJECT_NAME}_test_support)

  catkin_add_gtest(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests test/extrinsic_camera_on_wrist_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests ${PROJECT_NAME}_test_support)

  catkin_add_gtest(${PROJECT_NAME}_extrinsic_static_camera_robot_calibration_tests test/extrinsic_static_3d_camera_robot_calibration_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_extrinsic_static_camera_robot_calibration_tests ${PROJECT_NAME}_test_support)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
