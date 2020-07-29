find_package(GTest REQUIRED)
find_package(catkin REQUIRED COMPONENTS roslib)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_charuco_test charuco_test.cpp)
target_link_libraries(${PROJECT_NAME}_charuco_test
    PRIVATE
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest
    gtest_main
)
add_dependencies(${PROJECT_NAME}_charuco_test ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
