set(CMAKE_BUILD_TYPE DEBUG)



message("############################## GTEST ##############################")
#https://cmake.org/cmake/help/latest/module/FindGTest.html
#or cmake -DGTEST_ROOT=ENV{HOME}/usr
#set(GTEST_ROOT "$ENV{HOME}/usr" )
find_package(GTest REQUIRED)

include_directories(BEFORE ${GTEST_INCLUDE_DIRS})

MESSAGE("GTEST_FOUND: " ${GTEST_FOUND})
MESSAGE("GTEST_INCLUDE_DIRS: " ${GTEST_INCLUDE_DIRS})
MESSAGE("GTEST_LIBRARIES: " ${GTEST_LIBRARIES})
MESSAGE("GTEST_MAIN_LIBRARIES: " ${GTEST_MAIN_LIBRARIES})
MESSAGE("GTEST_BOTH_LIBRARIES: " ${GTEST_BOTH_LIBRARIES})


add_executable(kinematics_tests src/kinematics_tests.cpp ../../src/task.cpp)
target_link_libraries(kinematics_tests -pthread ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES}   )
gtest_discover_tests(kinematics_tests)



