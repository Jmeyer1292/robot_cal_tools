cmake_minimum_required(VERSION 2.8.3)
project(rct_image_tools)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rct_optimizations
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
    rct_optimizations
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/image_observation_finder.cpp
  src/${PROJECT_NAME}/aruco_finder.cpp
  src/${PROJECT_NAME}/circle_detector.cpp
  src/${PROJECT_NAME}/modified_circle_grid_target.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(${PROJECT_NAME}_test
  src/observation_finder_tests.cpp
)

add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

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
)
