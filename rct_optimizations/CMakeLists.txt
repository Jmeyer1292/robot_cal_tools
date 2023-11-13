cmake_minimum_required(VERSION 3.5.0)
project(rct_optimizations VERSION 0.1.0 LANGUAGES CXX)

find_package(rct_common REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  # Utilities
  src/${PROJECT_NAME}/eigen_conversions.cpp
  src/${PROJECT_NAME}/covariance_analysis.cpp
  # Optimizations (Simple)
  src/${PROJECT_NAME}/circle_fit.cpp
  # Optimizations (multiple cameras)
  src/${PROJECT_NAME}/extrinsic_multi_static_camera.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_only.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_wrist_only.cpp
  # Optimizations (extrinsic hand-eye, 2D and 3D cameras)
  src/${PROJECT_NAME}/extrinsic_hand_eye.cpp
  # Optimizations (Experimental) - Intrinsic
  src/${PROJECT_NAME}/camera_intrinsic.cpp
  src/${PROJECT_NAME}/pnp.cpp
  src/${PROJECT_NAME}/multi_camera_pnp.cpp
  # DH Chain
  src/${PROJECT_NAME}/dh_chain.cpp
  # Validation Tools
  src/${PROJECT_NAME}/validation/camera_intrinsic_calibration_validation.cpp
  # Noise Qualification
  src/${PROJECT_NAME}/validation/noise_qualification.cpp
  # Target Homography
  src/${PROJECT_NAME}/validation/homography_validation.cpp
  # DH Chain Kinematic Calibration
  src/${PROJECT_NAME}/dh_chain.cpp
  src/${PROJECT_NAME}/dh_chain_kinematic_calibration.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

if(RCT_BUILD_TESTS)
  enable_testing()
  rct_add_run_tests_target()
  add_subdirectory(test)
endif()

rct_configure_package(${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
