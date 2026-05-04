message("############################## GTEST ##############################")
find_package(GTest REQUIRED)
include(GoogleTest)

MESSAGE("GTest_FOUND: " ${GTest_FOUND})

add_executable(kinematics_tests src/kinematics_tests.cpp ../src/task.cpp)
target_link_libraries(kinematics_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
    Threads::Threads
)
gtest_discover_tests(kinematics_tests)



