# Add executable
add_executable(my_model_tester
  main.cpp
  test.cpp
)

# Link libraries
target_link_libraries(my_model_tester PUBLIC
  GTest::GTest
  GTest::Main
  sar
)

# Discover Google Tests
gtest_discover_tests(my_model_tester)
