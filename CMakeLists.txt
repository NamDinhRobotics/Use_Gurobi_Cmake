cmake_minimum_required(VERSION 3.2)

project(gurobi_ex C)

option(CXX "enable C++ compilation" ON)

if(CXX)
    enable_language(CXX)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})

# list source files here
set(sources mip1_c++.cpp)

add_executable(gurobi_ex main.cpp)
add_executable(mpc_test mpc.cpp)
add_executable(nlmpc nlmpc.cpp)

if(CXX)
    set(CMAKE_CXX_STANDARD 11)
    target_link_libraries(gurobi_ex optimized ${GUROBI_CXX_LIBRARY}
            debug ${GUROBI_CXX_DEBUG_LIBRARY})
    target_link_libraries(mpc_test optimized ${GUROBI_CXX_LIBRARY}
            debug ${GUROBI_CXX_DEBUG_LIBRARY})
    target_link_libraries(nlmpc optimized ${GUROBI_CXX_LIBRARY}
            debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(gurobi_ex ${GUROBI_LIBRARY})
target_link_libraries(mpc_test ${GUROBI_LIBRARY})
target_link_libraries(nlmpc ${GUROBI_LIBRARY})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    include(FeatureSummary)
    feature_summary(WHAT ALL)
endif()