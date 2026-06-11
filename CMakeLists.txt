#Copyright 2012 Kirill Van Heerden
#use "ccmake ." to change compile options

PROJECT(RobotJoint)
cmake_minimum_required(VERSION 2.8)
Set(CMAKE_MODULE_PATH CMake_Modules)


Set(RobotJoint_VERSION_MAJOR 0)
Set(RobotJoint_VERSION_MINOR 3)


OPTION(DEBUG_BUILD "Debug/Release build" OFF)
OPTION(CPU_OPTIMIZATIONS "Uses CPU optimizations like SSE for speed" ON)
OPTION(USE_FLOAT "Use floats instead of doubles?" OFF)
OPTION(TESTSUITE "Build the Test Suite" ON)
OPTION(RJ_THREADED "Enable Multi-Threading(requires c++0x support)" OFF)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake_Modules/RJointTypes.h.in ${CMAKE_CURRENT_BINARY_DIR}/Include/RJointTypes.h)

#Needed for LAPACK
enable_language(Fortran )


#optimizations
if(CPU_OPTIMIZATIONS)
include(CMake_Modules/OptimizeForArchitecture.cmake)
OptimizeForArchitecture()
ENDIF()

if(RJ_THREADED)
#set c++ 11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ENDIF()

find_package (Threads)
find_package (LAPACK REQUIRED )
#find_package (Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(Include TestSuiteFiles)

set(src_dir src)
set(hdr_dir Include)
set(test_dir TestSuiteFiles)

file (GLOB ROBOTJOINT_SOURCE "${src_dir}/*.cxx")
file (GLOB ROBOTJOINT_INCLUDE "${hdr_dir}/*.h")
file (GLOB ROBOTJOINT_TESTS "${test_dir}/*.cxx")
    
ADD_LIBRARY(RobotJoint
        ${ROBOTJOINT_SOURCE}
)

if(TESTSUITE)
        add_executable(TestSuite
                ${ROBOTJOINT_TESTS}
        )
        TARGET_LINK_LIBRARIES(TestSuite
                RobotJoint
                lapack
        )
ENDIF()

# Documentation generation command
# Credits to Stefan Majewsky from the "The Geek Shall Inherit The Earth" blog 
# (https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/)
find_package(Doxygen)
if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake_Modules/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
endif(DOXYGEN_FOUND)

#Installation commands
        #library
                INSTALL(TARGETS RobotJoint
                ARCHIVE DESTINATION lib
                )
#                 EXPORT RobotJoint-targets
        #headers
                INSTALL(DIRECTORY Include/ DESTINATION include/RobotJoint)
        #project export
#                 INSTALL(EXPORT RobotJoint-targets DESTINATION include)

        # uninstall target
                configure_file(
                "${CMAKE_CURRENT_SOURCE_DIR}/CMake_Modules/cmake_uninstall.cmake.in"
                "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
                IMMEDIATE @ONLY)

                add_custom_target(uninstall
                COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
                )


#enable compiler optimization
        IF(DEBUG_BUILD)
                set(CMAKE_BUILD_TYPE Debug)
        ELSE(DEBUG_BUILD)
                set(CMAKE_BUILD_TYPE Release)
        ENDIF()
