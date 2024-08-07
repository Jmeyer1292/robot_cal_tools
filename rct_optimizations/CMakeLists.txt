cmake_minimum_required(VERSION 3.16.0)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

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
  # Projection
  src/${PROJECT_NAME}/validation/projection.cpp
  # DH Chain Kinematic Calibration
  src/${PROJECT_NAME}/dh_chain.cpp
  src/${PROJECT_NAME}/dh_chain_kinematic_calibration.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${CERES_LIBRARIES}
         rct::rct_common
         Boost::boost
         yaml-cpp)

if(RCT_BUILD_TESTS)
  enable_testing()
  add_run_tests_target(ENABLE ${RCT_ENABLE_RUN_TESTS})
  add_subdirectory(test)
endif()

configure_package(
  NAMESPACE rct
  TARGETS ${PROJECT_NAME}
          DEPENDENCIES
          Boost
          rct_common
          yaml-cpp
          Ceres)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
