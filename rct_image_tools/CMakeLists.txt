cmake_minimum_required(VERSION 2.8.3)
project(rct_image_tools)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rct_optimizations
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rct_image_tools
  CATKIN_DEPENDS
    cv_bridge
    rct_optimizations
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
  src/${PROJECT_NAME}/circle_detector.cpp
  src/${PROJECT_NAME}/modified_circle_grid_target.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test
  src/observation_finder_tests.cpp
)

add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Executable testing the PnP problem solutions
add_executable(${PROJECT_NAME}_pnp
  src/solve_pnp.cpp
)

add_dependencies(${PROJECT_NAME}_pnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pnp
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${CERES_LIBRARIES}
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
  PATTERN ".svn" EXCLUDE
)
