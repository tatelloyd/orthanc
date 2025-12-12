cmake_minimum_required(VERSION 3.16)
project(orthanc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/behaviortree_cpp")

# Find packages
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Include directories
include_directories(${CURL_INCLUDE_DIRS})

# Source files
add_executable(orthanc_tracker
    src/cpp/main.cpp
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

# Link libraries
target_link_libraries(orthanc_tracker
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    Threads::Threads
    pigpiod_if2
    #${BEHAVIORTREE_LIBRARY}
    #BT::behaviortree_cpp
)

add_executable(bt_tracker
    src/cpp/bt_tracker_main.cpp
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

# Link libraries
target_link_libraries(bt_tracker
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    Threads::Threads
    pigpiod_if2
    ${BEHAVIORTREE_LIBRARY}
)

# Copy tree files to build directory
file(COPY ${CMAKE_SOURCE_DIR}/src/cpp/trees
     DESTINATION ${CMAKE_BINARY_DIR})

# Build interactive menu (for testing)
add_executable(orthanc_menu
    tests/cpp/menu.cpp
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

target_link_libraries(orthanc_menu
    Threads::Threads
    pigpiod_if2
)