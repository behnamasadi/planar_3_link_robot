cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

project(planar_3_link_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

MESSAGE("Eigen3_FOUND: " ${Eigen3_FOUND})
MESSAGE("Eigen3_VERSION: " ${Eigen3_VERSION})



#set warning level into errors 
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
endif()

#Finding Memory leaking, Stack and Heap overflow
#set(CMAKE_CXX_FLAGS "-fsanitize=address ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-fno-omit-frame-pointer ${CMAKE_CXX_FLAGS}")


add_executable(main src/main.cpp src/task.cpp)
target_link_libraries(main PRIVATE Eigen3::Eigen)


option(TESTING_ENABLED "Enable testing" ON)
if(TESTING_ENABLED)
    enable_testing()
    add_subdirectory(tests)
endif()
