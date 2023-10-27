
cmake_minimum_required( VERSION 3.13 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "build type" FORCE)
endif()

project( p1ac )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( PoseLib REQUIRED )

include_directories(${CMAKE_SOURCE_DIR})

add_library(p1ac
    p1ac/p1ac.h p1ac/p1ac.cpp
)
target_link_libraries(p1ac PRIVATE PoseLib::PoseLib)

add_subdirectory(examples)

set_target_properties(p1ac PROPERTIES PUBLIC_HEADER "p1ac/p1ac.h")
INSTALL(TARGETS p1ac 
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/p1ac
)

