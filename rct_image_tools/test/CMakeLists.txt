find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_charuco_test charuco_test.cpp)
target_link_libraries(${PROJECT_NAME}_charuco_test
    PRIVATE
    ${PROJECT_NAME}
    gtest
    gtest_main
)
add_dependencies(${PROJECT_NAME}_charuco_test ${PROJECT_NAME})
