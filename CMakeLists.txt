cmake_minimum_required(VERSION 3.6)
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
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake module directory (for FindPCAP)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# SLAM support (not Windows compatible)
option(WITH_SLAM "Build SLAM algorithms" ON) # Enabled by default
if(WITH_SLAM)
    add_definitions(-DWITH_SLAM)
    message(STATUS "Building with SLAM support")
else()
    message(STATUS "Building without SLAM support")
endif()

# Set Visual Studio startUp project
# (Requires CMake 3.6.0 RC1 or later)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "olp")

# Thread
find_package(Threads REQUIRED)

# Boost
find_package(Boost 1.60 REQUIRED filesystem log program_options system thread)
include_directories(${Boost_INCLUDE_DIR})
link_libraries(${Boost_LIBRARIES})

# PCL
# (BOOST_INCLUDEDIR required if not on default Linux location /usr/include)
set(BOOST_INCLUDEDIR ${Boost_INCLUDE_DIR})
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# PCAP
find_package(PCAP)

# LAStools
find_package(LASlib PATHS vendor/LAStools/install)
if(LASlib_FOUND)
    set(LASlib_LIBRARY_DIRS ${LASlib_INCLUDE_DIRS}/../../lib/LASlib/)
    set(LASlib_LIBRARIES LASlib)
else()
    if(MSVC AND EXISTS ${CMAKE_SOURCE_DIR}/vendor/LAStools/install/include)
        set(LASlib_FOUND true)
        set(LASlib_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/vendor/LAStools/install/include/LASlib)
        set(LASlib_LIBRARY_DIRS ${CMAKE_SOURCE_DIR}/vendor/LAStools/install/lib/LASlib/Release)
        set(LASlib_LIBRARIES LASlib)
        message(STATUS "LASlib found by manual lookup")
    endif()
endif()
if(NOT LASlib_FOUND)
    message(SEND_ERROR "LASlib not found")
endif()
include_directories(${LASlib_INCLUDE_DIRS})
link_directories(${LASlib_LIBRARY_DIRS})

# libpointmatcher
find_package(libnabo REQUIRED PATHS vendor/libnabo/install)
find_package(libpointmatcher REQUIRED PATHS vendor/libpointmatcher/install)
include_directories(${libpointmatcher_INCLUDE_DIRS})

# ikalman
include_directories(vendor/ikalman)
add_library(ikalman
    vendor/ikalman/kalman.c vendor/ikalman/kalman.h
    vendor/ikalman/matrix.c vendor/ikalman/matrix.h
    vendor/ikalman/gps.c vendor/ikalman/gps.h)

# TinyEKF
include_directories(vendor/TinyEKF/src)
add_library(tinyekf
    vendor/TinyEKF/src/tiny_ekf.c)

# Targets
add_executable(olp
    main.cpp
    viewer.hpp
    helpers/LASHelper.cpp helpers/LASHelper.h
    helpers/FileHelper.cpp helpers/FileHelper.h
    helpers/GPSHelper.cpp helpers/GPSHelper.h
    helpers/KalmanHelper.cpp helpers/KalmanHelper.h
    helpers/TinyEKFHelper.cpp helpers/TinyEKFHelper.h
    helpers/CsvFileWriter.h
    helpers/Utility.h
    helpers/ViewerHelper.h
    helpers/GPSExtrapolationHelper.hpp
    piping/ProcessorPipe.hpp
    compute/Processor.h
    compute/Producer.hpp
    compute/Consumer.hpp
    compute/Filter.hpp
    compute/Transformer.hpp
    compute/calculators/Calculator.h
    compute/calculators/GPSCalculator.hpp
    compute/calculators/GPSPacketCalculator.hpp
    compute/calculators/ICPSLAMCalculator.hpp
    compute/calculators/LOAMSLAMCalculator.hpp
    grabber/GPSPacket.h
    grabber/GPSVLPGrabber.h grabber/GPSVLPGrabber.cpp
    grabber/NmeaParser.h grabber/NmeaParser.cpp
)

target_link_libraries(olp
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PCAP_LIBRARY}
    ${LASlib_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    ${libnabo_LIBRARIES}
    ikalman
    tinyekf)

install(TARGETS olp
    DESTINATION ${CMAKE_INSTALL_PREFIX})
