cmake_minimum_required(VERSION 3.5.0)
project(rct_image_tools VERSION 0.1.0 LANGUAGES CXX)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rct_common REQUIRED)
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

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/circle_detector.cpp
  src/${PROJECT_NAME}/modified_circle_grid_target.cpp
  src/${PROJECT_NAME}/modified_circle_grid_finder.cpp
  src/${PROJECT_NAME}/aruco_grid_target.cpp
  src/${PROJECT_NAME}/aruco_finder.cpp
  src/${PROJECT_NAME}/charuco_grid_target.cpp
  src/${PROJECT_NAME}/charuco_finder.cpp
)
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  opencv_core
  opencv_aruco
  opencv_features2d
  opencv_imgproc
  opencv_highgui
  yaml-cpp
  Eigen3::Eigen
  rct::rct_optimizations
)

# Modified circle grid finder executable
# TODO: turn this into a unit test
add_executable(${PROJECT_NAME}_test
  src/target_finder_tests.cpp
)
target_compile_options(${PROJECT_NAME}_test PUBLIC -std=c++11)
target_compile_options(${PROJECT_NAME}_test PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_test PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  rct::rct_optimizations
)

if(RCT_BUILD_TESTS)
  enable_testing()
  rct_add_run_tests_target()
  add_subdirectory(test)
endif()

rct_configure_package(${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
