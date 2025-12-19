cmake_minimum_required(VERSION 3.16)
project(orthanc VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BehaviorTree.CPP path
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/behaviortree_cpp")

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(${CURL_INCLUDE_DIRS})

# ============================================================================
# Source Files (shared across executables)
# ============================================================================
set(ORTHANC_CORE_SOURCES
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

# ============================================================================
# Executables
# ============================================================================

# Main BehaviorTree-based tracker (production)
add_executable(bt_tracker
    src/cpp/bt_tracker_main.cpp
    ${ORTHANC_CORE_SOURCES}
)

target_link_libraries(bt_tracker
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    Threads::Threads
    pigpiod_if2
    BT::behaviortree_cpp
)

# Interactive testing menu
add_executable(orthanc_menu
    tests/cpp/menu.cpp
    ${ORTHANC_CORE_SOURCES}
)

target_link_libraries(orthanc_menu
    Threads::Threads
    pigpiod_if2
)

# ============================================================================
# Copy Resources to Build Directory
# ============================================================================
# Copy BehaviorTree XML files
file(COPY ${CMAKE_SOURCE_DIR}/src/cpp/trees
     DESTINATION ${CMAKE_BINARY_DIR})

# ============================================================================
# Installation (Optional - for system-wide deployment)
# ============================================================================
install(TARGETS bt_tracker orthanc_menu
    RUNTIME DESTINATION bin
)

install(DIRECTORY src/cpp/trees
    DESTINATION share/orthanc
)