cmake_minimum_required(VERSION 3.15)
project(Orthanc VERSION 1.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Find required libraries
find_library(PIGPIO_LIBRARY pigpio REQUIRED)
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 REQUIRED)
find_package(Threads REQUIRED)

# Optional: Find Python for embedding (if needed later)
# find_package(Python3 COMPONENTS Interpreter Development)

# Source files
set(TURRET_SOURCES
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

set(TURRET_HEADERS
    src/cpp/Turret.hpp
    src/cpp/ServoController.hpp
    src/cpp/SignalGenerator.hpp
)

# Main executable
add_executable(orthanc
    src/cpp/main.cpp
    ${TURRET_SOURCES}
)

target_include_directories(orthanc 
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

target_link_libraries(orthanc 
    PRIVATE 
        ${PIGPIO_LIBRARY}
        ${PIGPIOD_IF2_LIBRARY}
        Threads::Threads
)

# Set output directory
set_target_properties(orthanc PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin
)

# Optional: Build as library for reuse (good for multi-turret setup)
option(BUILD_SHARED_LIBS "Build shared library" OFF)

if(BUILD_SHARED_LIBS)
    add_library(orthanc_lib SHARED ${TURRET_SOURCES})
    target_include_directories(orthanc_lib
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
    )
    target_link_libraries(orthanc_lib
        PUBLIC
            ${PIGPIO_LIBRARY}
            ${PIGPIOD_IF2_LIBRARY}
            Threads::Threads
    )
endif()

# Tests
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
    enable_testing()
    
    add_executable(test_servo
        tests/cpp/test_servo.cpp
        src/cpp/ServoController.cpp
    )
    
    target_include_directories(test_servo 
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
    )
    
    target_link_libraries(test_servo
        PRIVATE
            ${PIGPIO_LIBRARY}
            ${PIGPIOD_IF2_LIBRARY}
            Threads::Threads
    )
    
    set_target_properties(test_servo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin
    )
    
    add_test(NAME ServoTest COMMAND test_servo)
endif()

# Installation rules
install(TARGETS orthanc
    RUNTIME DESTINATION bin
)

if(BUILD_SHARED_LIBS)
    install(TARGETS orthanc_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    install(FILES ${TURRET_HEADERS}
        DESTINATION include/orthanc
    )
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "Orthanc Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
message(STATUS "  Build shared library: ${BUILD_SHARED_LIBS}")
message(STATUS "")