include_directories(.)

add_subdirectory(problem_generator)

add_executable(test_random_problems test_random_problems.cpp)
target_link_libraries(test_random_problems p1ac problem_generator)

