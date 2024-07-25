cmake_minimum_required(VERSION 3.16.0)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rct_optimizations REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_library(
  ${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/circle_detector.cpp
  src/${PROJECT_NAME}/modified_circle_grid_target.cpp
  src/${PROJECT_NAME}/modified_circle_grid_finder.cpp
  src/${PROJECT_NAME}/aruco_grid_target.cpp
  src/${PROJECT_NAME}/aruco_finder.cpp
  src/${PROJECT_NAME}/charuco_grid_target.cpp
  src/${PROJECT_NAME}/charuco_finder.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::boost
         Boost::filesystem
         opencv_core
         opencv_aruco
         opencv_features2d
         opencv_imgproc
         opencv_highgui
         yaml-cpp
         Eigen3::Eigen
         rct::rct_optimizations)

# Modified circle grid finder executable TODO: turn this into a unit test
add_executable(${PROJECT_NAME}_test src/target_finder_tests.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_compile_options(${PROJECT_NAME}_test PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_test PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} rct::rct_optimizations)

if(RCT_BUILD_TESTS)
  enable_testing()
  add_run_tests_target(ENABLE ${RCT_ENABLE_RUN_TESTS})
  add_subdirectory(test)
endif()

configure_package(
  NAMESPACE rct
  TARGETS ${PROJECT_NAME}
          ${PROJECT_NAME}_test
          DEPENDENCIES
          "Boost REQUIRED COMPONENTS filesystem"
          OpenCV
          rct_optimizations
          yaml-cpp)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
