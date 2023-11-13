find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_target_finder_test target_finder_utest.cpp)
target_link_libraries(${PROJECT_NAME}_target_finder_test PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_target_finder_test)
add_dependencies(${PROJECT_NAME}_target_finder_test ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_target_finder_test)
target_compile_definitions(${PROJECT_NAME}_target_finder_test
                           PUBLIC TEST_SUPPORT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/images/")

# Install the test executables so they can be run independently later if needed
install(
  TARGETS ${PROJECT_NAME}_target_finder_test
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests)
