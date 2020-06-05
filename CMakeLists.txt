cmake_minimum_required(VERSION 2.8)
project(olp)

# Set a default build type if none was specified
set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 14)

# Targets
add_executable(olp
    main.cpp
    viewer.hpp
    processor.hpp)

# Set Visual Studio startUp project
# (Requires CMake 3.6.0 RC1 or later)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "olp")

# Thread
find_package(Threads REQUIRED)

# PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
# Additional include directories
include_directories(${PCL_INCLUDE_DIRS})
# Preprocessor definitions
add_definitions(${PCL_DEFINITIONS})
# For disabling precompiled features
#add_definitions(-DPCL_NO_PRECOMPILE)
# Additional library directories
link_directories(${PCL_LIBRARY_DIRS})
# Additional dependencies
target_link_libraries(olp ${PCL_LIBRARIES})
