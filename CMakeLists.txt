cmake_minimum_required(VERSION 3.20)
project(gurobi1)

add_definitions(-std=c++11)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(GUROBI REQUIRED)

if(GUROBI_FOUND)
    message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
    message(FATAL_ERROR "GUROBI NOT FOUND!")
endif(GUROBI_FOUND)


set(CMAKE_BUILD_TYPE Release) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo)

include_directories(${GUROBI_INCLUDE_DIRS})
#set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )

FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};$ENV{GUROBI_HOME}/lib/" )

add_executable(gurobi1 main.cpp)
target_link_libraries(gurobi1 ${PCL_LIBRARIES} ${GUROBI_LIBRARIES})