cmake_minimum_required(VERSION 3.10)
project(BlindTrackVision VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Option to enable ZED SDK support
option(USE_ZED_SDK "Enable ZED SDK support" OFF)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Output OpenCV version and include directories for debugging
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

# Define source files
file(GLOB SOURCES "src/*.cpp")

# Define compile definitions and libraries
set(ADDITIONAL_LIBRARIES "")
set(COMPILE_DEFINITIONS "")

# Check if ZED SDK is enabled
if(USE_ZED_SDK)
    # Find ZED SDK package
    find_package(ZED 3 QUIET)
    if(ZED_FOUND)
        message(STATUS "ZED SDK found - enabling ZED camera support")
        include_directories(${ZED_INCLUDE_DIRS})
        link_directories(${ZED_LIBRARY_DIR})
        set(ADDITIONAL_LIBRARIES ${ZED_LIBRARIES} ${CUDA_LIBRARIES})
        set(COMPILE_DEFINITIONS USE_ZED_SDK)
        
        # Find CUDA (required by ZED SDK)
        find_package(CUDA REQUIRED)
        include_directories(${CUDA_INCLUDE_DIRS})
        
        message(STATUS "ZED include dirs: ${ZED_INCLUDE_DIRS}")
        message(STATUS "ZED library dir: ${ZED_LIBRARY_DIR}")
        message(STATUS "CUDA include dirs: ${CUDA_INCLUDE_DIRS}")
    else()
        message(WARNING "ZED SDK not found - disabling ZED camera support")
    endif()
endif()

# Add compile definitions
add_definitions(-D${COMPILE_DEFINITIONS})

# Create executable
add_executable(blind_track_vision ${SOURCES})

# Link libraries
target_link_libraries(blind_track_vision ${OpenCV_LIBS} ${ADDITIONAL_LIBRARIES})

# Add thread support
find_package(Threads REQUIRED)
target_link_libraries(blind_track_vision ${CMAKE_THREAD_LIBS_INIT})

# Copy configuration files to build directory
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR}) 