#Copyright 2012 Kirill Van Heerden
#use "ccmake ." to change compile options

PROJECT(ApliRobotJoint)
cmake_minimum_required(VERSION 2.8)
Set(CMAKE_MODULE_PATH CMake_Modules)

#Needed by Lapack
enable_language(Fortran ) 

find_package (LibRobotJoint REQUIRED)
find_package (LAPACK REQUIRED )

INCLUDE_DIRECTORIES(${LIBROBOTJOINT_INCLUDE_DIRS})

# set(src_dir src)
# set(hdr_dir Include)

# file (GLOB ROBOTJOINT_SOURCE "${src_dir}/*.cxx")
# file (GLOB ROBOTJOINT_INCLUDE "${hdr_dir}/*.h")



add_executable(TestSuite
        main.cxx
)
TARGET_LINK_LIBRARIES(TestSuite
        ${LIBROBOTJOINT_LIBRARIES}
        lapack
)

set(CMAKE_BUILD_TYPE Debug)